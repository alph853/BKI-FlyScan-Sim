import cv2
import pandas as pd
from QR_processing import QRProcessor

db = pd.DataFrame([
    {"ma_san_pham": 1, "noi_dung": "https://qr.zalo.me/"},
    {"ma_san_pham": 2, "noi_dung": "https://get-qr.com/EMzu3e"},
    {"ma_san_pham": 3, "noi_dung": "QC7-6203-000/11A/N686/250420/102/240/A/008/"},
    {"ma_san_pham": 4, "noi_dung": "https://www.yoyostamp.com/app"},
    {"ma_san_pham": 5, "noi_dung": "SPXVN056155068825"},
    {"ma_san_pham": 6, "noi_dung": "QC7-6203-000/11A/N686/250420/105/240/A/008/"},
    {"ma_san_pham": 7, "noi_dung": "QC7-6203-000/11A/N686/250420/103/240/A/008/"},
    {"ma_san_pham": 8, "noi_dung": "QC7-6203-000/11A/N686/250420/104/240/A/008/"},
    {"ma_san_pham": 9, "noi_dung": "QC7-6203-000/11A/N686/250420/039/240/A/008/"},
    {"ma_san_pham": 10, "noi_dung": "QC7-6203-000/11A/N686/250420/036/240/A/008/"},
])

processor = QRProcessor(database=db, id_col='ma_san_pham', data_col='noi_dung')

RTMP_URL = "rtmp://192.168.33.108:1935/live/stream"
OUTPUT_CSV = "output.csv"

cap = cv2.VideoCapture(RTMP_URL)
if not cap.isOpened():
    print("ERROR: cannot open camera")
    exit(1)

print("Starting video capture... (press Enter to process, 'q' to quit)")

with open(OUTPUT_CSV, "w") as f:
    f.write("ma_san_pham,noi_dung\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: failed to grab frame")
        break

    display = processor.show_frame(frame.copy())
    cv2.imshow("frame", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    if key == 13:  # Enter
        proc_frame = frame.copy()

        loading = proc_frame.copy()
        cv2.putText(
            loading, "Processing...", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
        )
        cv2.imshow("frame", loading)
        cv2.waitKey(1)

        processed = processor.process_frame(proc_frame)

        result_img = proc_frame.copy()
        for p in processed:
            x1, y1, x2, y2 = map(int, p["bbox"])
            cv2.rectangle(result_img,
                          (x1, y1), (x2, y2),
                          (0, 255, 0), 2)
            cv2.putText(result_img,
                        p["data"],
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 0), 2)

        cv2.imshow("frame", result_img)

        print("Press 'y' to save results, 'n' to discard, or 'q' to quit.")
        while True:
            k = cv2.waitKey(0) & 0xFF
            if not processed:
                print("No predictions found")
                input("Press Enter to continue...")
                break

            if k == ord('y'):
                with open("output.csv", "a") as f:
                    for p in processed:
                        f.write(f"1,{p['data']}\n")  # dummy index
                print("Results saved to output.csv")
                break
            elif k == ord('n'):
                print("Results discarded")
                break
            elif k == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                exit(0)

cap.release()
cv2.destroyAllWindows()
