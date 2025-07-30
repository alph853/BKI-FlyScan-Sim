#!/usr/bin/env python3
"""
Script to convert YOLOv8 .pt model to ONNX format for C++ inference
"""

import torch
from ultralytics import YOLO
import onnx
import onnxruntime as ort
import numpy as np
import argparse
import os

def convert_yolo_to_onnx(input_path, output_path, img_size=640, simplify=True):
    """
    Convert YOLOv8 .pt model to ONNX format
    
    Args:
        input_path (str): Path to .pt model file
        output_path (str): Path for output .onnx file
        img_size (int): Input image size (default: 640)
        simplify (bool): Whether to simplify the ONNX model
    """
    
    print(f"Loading YOLOv8 model from: {input_path}")
    
    # Load the YOLOv8 model
    model = YOLO(input_path)
    
    # Export to ONNX
    print(f"Converting to ONNX format...")
    print(f"Image size: {img_size}x{img_size}")
    
    # Export the model
    model.export(
        format='onnx',
        imgsz=img_size,
        simplify=simplify,
        dynamic=False,  # Static input shape for better C++ compatibility
        opset=11  # ONNX opset version
    )
    
    # The export method creates the ONNX file with the same name as input but .onnx extension
    generated_onnx_path = input_path.replace('.pt', '.onnx')
    
    # Move to desired output path if different
    if output_path != generated_onnx_path:
        import shutil
        shutil.move(generated_onnx_path, output_path)
        print(f"ONNX model moved to: {output_path}")
    else:
        print(f"ONNX model saved as: {output_path}")
    
    # Verify the ONNX model
    print("Verifying ONNX model...")
    try:
        onnx_model = onnx.load(output_path)
        onnx.checker.check_model(onnx_model)
        print("✓ ONNX model is valid")
        
        # Print model info
        print(f"\nModel Information:")
        print(f"Input shape: {onnx_model.graph.input[0].type.tensor_type.shape}")
        print(f"Output shapes:")
        for i, output in enumerate(onnx_model.graph.output):
            print(f"  Output {i}: {output.type.tensor_type.shape}")
            
    except Exception as e:
        print(f"✗ ONNX model verification failed: {e}")
        return False
    
    # Test inference with ONNX Runtime
    print("\nTesting ONNX Runtime inference...")
    try:
        # Create inference session
        ort_session = ort.InferenceSession(output_path)
        
        # Get input details
        input_name = ort_session.get_inputs()[0].name
        input_shape = ort_session.get_inputs()[0].shape
        print(f"Input name: {input_name}")
        print(f"Input shape: {input_shape}")
        
        # Create dummy input
        dummy_input = np.random.randn(1, 3, img_size, img_size).astype(np.float32)
        
        # Run inference
        outputs = ort_session.run(None, {input_name: dummy_input})
        print(f"✓ Inference successful!")
        print(f"Output shapes: {[output.shape for output in outputs]}")
        
        # Print COCO class names for reference
        # print(f"\nCOCO Classes (0-79):")
        # coco_classes = [
        #     'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
        #     'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
        #     'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
        #     'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
        #     'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        #     'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        #     'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
        #     'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
        #     'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
        #     'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        # ]
        
        # for i, cls_name in enumerate(coco_classes):
        #     print(f"  {i}: {cls_name}")
        
    except Exception as e:
        print(f"✗ ONNX Runtime test failed: {e}")
        return False
    
    print(f"\n✓ Conversion completed successfully!")
    print(f"ONNX model ready for C++ inference: {output_path}")
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Convert YOLOv8 .pt model to ONNX')
    parser.add_argument('--input', '-i', 
                       default='src/flyscan_perception/yolov8n.pt',
                       help='Input .pt model path')
    parser.add_argument('--output', '-o', 
                       default='src/flyscan_perception/yolov8n.onnx',
                       help='Output .onnx model path')
    parser.add_argument('--img-size', type=int, default=640,
                       help='Input image size (default: 640)')
    parser.add_argument('--no-simplify', action='store_true',
                       help='Disable ONNX model simplification')
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input):
        print(f"Error: Input file not found: {args.input}")
        return 1
    
    # Create output directory if needed
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Convert the model
    success = convert_yolo_to_onnx(
        args.input, 
        args.output, 
        args.img_size, 
        not args.no_simplify
    )
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())