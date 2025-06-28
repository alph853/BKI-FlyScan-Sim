import os
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from ultralytics import YOLO
import pandas as pd
from concurrent.futures import ThreadPoolExecutor, as_completed


class QRProcessor:
    def __init__(self, model_path="best.pt", conf=0.25, imgsz=640, margin_factor=0.2, upscale_factor=2.0, database=None, id_col='ID', data_col='data'):
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.margin_factor = margin_factor
        self.upscale_factor = upscale_factor
        self.db_data = None
        if database is not None:
            self.db_data = self._load_database(database, id_col, data_col)
        os.makedirs("qr_crops", exist_ok=True)

    def _load_database(self, db, id_col, data_col):
        if isinstance(db, pd.DataFrame):
            df = db.rename(columns={id_col: 'ID', data_col: 'data'})
        elif isinstance(db, list):
            df = pd.DataFrame(db).rename(columns={id_col: 'ID', data_col: 'data'})
        else:
            raise ValueError("Unsupported database format. Use DataFrame or list of dicts.")
        df['data'] = df['data'].astype(str)
        return set(df['data'].values)

    def _is_valid_data(self, data: str) -> bool:
        if (self.db_data is None) or (data in self.db_data):
            print("Valid data", data)
            return True
        print("Invalid data", data)
        return False

    def _sharpen_image(self, image: np.ndarray) -> np.ndarray:
        blurred = cv2.GaussianBlur(image, (0, 0), sigmaX=3, sigmaY=3)
        return cv2.addWeighted(image, 1.5, blurred, -0.5, 0)

    def _deskew_qr(self, img: np.ndarray) -> np.ndarray:
        detector = cv2.QRCodeDetector()
        ok, points = detector.detect(img)
        if not ok or points is None:
            raise ValueError("Could not detect QR code corners")

        pts = points[0].astype(np.float32)
        distances = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        side = int(max(distances))

        dest = np.array([[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32)
        transform = cv2.getPerspectiveTransform(pts, dest)
        return cv2.warpPerspective(img, transform, (side, side))

    def _try_decode(self, img):
        decoded_objs = decode(img)
        for obj in decoded_objs:
            data = obj.data.decode(errors="ignore")
            if self._is_valid_data(data):
                return [obj], img, {"method": "original"}
        return None

    def _try_decode_variant(self, img, alpha, beta, clahe_clip, clahe_grid, angle):
        adjusted = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
        lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_grid)
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        sharpened = self._sharpen_image(enhanced)

        if angle != 0:
            M = cv2.getRotationMatrix2D((sharpened.shape[1] // 2, sharpened.shape[0] // 2), angle, 1.0)
            img_to_decode = cv2.warpAffine(sharpened, M, (sharpened.shape[1], sharpened.shape[0]))
        else:
            img_to_decode = sharpened

        decoded_objs = decode(img_to_decode)
        for obj in decoded_objs:
            data = obj.data.decode(errors="ignore")
            if self._is_valid_data(data):
                return [obj], img_to_decode, {
                    'alpha': round(alpha, 2),
                    'beta': int(beta),
                    'angle': angle,
                    'clahe_clip': clahe_clip,
                    'clahe_grid': clahe_grid
                }
        return None

    def _adjust_and_decode_parallel(self, img: np.ndarray) -> tuple:
        result = self._try_decode(img)
        if result:
            return result

        clahe_settings = [
            {'clip_limit': 2.0, 'grid_size': (8, 8)},
            {'clip_limit': 3.0, 'grid_size': (8, 8)},
            {'clip_limit': 2.0, 'grid_size': (16, 16)},
            {'clip_limit': 3.0, 'grid_size': (16, 16)},
        ]
        alphas = np.arange(0.5, 3.1, 0.5)
        betas = np.arange(-60, 61, 20)
        angles = [0, 90, 180, 270]

        tasks = []
        with ThreadPoolExecutor(max_workers=8) as executor:
            for setting in clahe_settings:
                for alpha in alphas:
                    for beta in betas:
                        for angle in angles:
                            tasks.append(
                                executor.submit(
                                    self._try_decode_variant,
                                    img, alpha, beta,
                                    setting['clip_limit'],
                                    setting['grid_size'],
                                    angle
                                )
                            )
            for future in as_completed(tasks):
                result = future.result()
                if result:
                    return result

        return None, None, None

    def show_frame(self, image: np.ndarray) -> np.ndarray:
        results = self.model.predict(image, conf=self.conf, imgsz=self.imgsz)
        predictions = results[0].boxes
        if predictions is None or len(predictions) == 0:
            return image

        out = image.copy()  
        for prediction in predictions:
            xyxy = prediction.xyxy.cpu().numpy().astype(int)[0]
            cv2.rectangle(
                out,
                (xyxy[0], xyxy[1]),
                (xyxy[2], xyxy[3]),
                (0, 255, 0),
                2
            )
        return out


    def process_frame(self, image: np.ndarray):
        height, width = image.shape[:2]
        results = self.model.predict(image, conf=self.conf, imgsz=self.imgsz)
        predictions = results[0].boxes
        if predictions is None or len(predictions) == 0:
            return []

        output = []
        for idx, box in enumerate(predictions, start=1):
            xyxy = box.xyxy.cpu().numpy().astype(int)[0]
            x1, y1, x2, y2 = xyxy
            w, h = x2 - x1, y2 - y1

            dx = int(w * self.margin_factor / 2)
            dy = int(h * self.margin_factor / 2)
            x1m, y1m = max(0, x1 - dx), max(0, y1 - dy)
            x2m, y2m = min(width, x2 + dx), min(height, y2 + dy)

            roi = image[y1m:y2m, x1m:x2m]
            if roi.size == 0:
                continue

            early_result = self._try_decode(roi)
            if early_result:
                decoded_objs, final_img, params = early_result
                output.append({
                    'index': idx,
                    'data': decoded_objs[0].data.decode(errors="ignore"),
                    'params': params,
                    'image': final_img,
                    'bbox': (x1m, y1m, x2m, y2m),
                    'saved_crop': f"qr_crops/roi_{idx}.png"
                })
                return output

            if self.upscale_factor != 1.0:
                roi = cv2.resize(
                    roi,
                    (int(roi.shape[1] * self.upscale_factor), int(roi.shape[0] * self.upscale_factor)),
                    interpolation=cv2.INTER_LANCZOS4
                )

            roi_path = f"qr_crops/roi_{idx}.png"
            cv2.imwrite(roi_path, roi)

            try:
                square = self._deskew_qr(roi)
            except ValueError:
                square = roi

            decoded_objs, final_img, params = self._adjust_and_decode_parallel(square)

            if decoded_objs:
                data = decoded_objs[0].data.decode(errors="ignore")
                output.append({
                    'index': idx,
                    'data': data,
                    'params': params,
                    'image': final_img,
                    'bbox': (x1m, y1m, x2m, y2m),
                    'saved_crop': roi_path
                })
                return output

        # Final fallback: decode original image
        final_decode = decode(image)
        for obj in final_decode:
            data = obj.data.decode(errors="ignore")
            if self._is_valid_data(data):
                return [{
                    'index': -1,
                    'data': data,
                    'params': {"method": "final_fallback_whole_image"},
                    'image': image,
                    'bbox': (0, 0, width, height),
                    'saved_crop': None
                }]

        return []