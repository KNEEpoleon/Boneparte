#!/usr/bin/env python3
"""
Generate ArUco Marker for ArucoTransform App

Creates a 15cm ArUco marker (ID=0, DICT_6X6_250) for Vision Pro localization.
The marker should be printed at exactly 15cm x 15cm physical size.

Usage:
    python3 generate_aruco_marker.py

Output:
    aruco_marker_id0_15cm.png - Print this at 15cm x 15cm
"""

import cv2
import numpy as np
import sys

def generate_aruco_marker(marker_id=0, marker_size_px=700, output_file='aruco_marker_id0_15cm.png'):
    """
    Generate ArUco marker image
    
    Args:
        marker_id: ArUco marker ID (default: 0)
        marker_size_px: Image size in pixels (default: 700)
        output_file: Output filename
    """
    print("=" * 60)
    print("ArUco Marker Generator")
    print("=" * 60)
    print(f"Marker ID: {marker_id}")
    print(f"Dictionary: DICT_6X6_250")
    print(f"Image size: {marker_size_px}x{marker_size_px} pixels")
    print(f"Physical size: 15cm x 15cm (MUST be exact)")
    print("=" * 60)
    
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    
    # Generate marker image
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
    
    # Add white border for better detection
    border_size = 25
    bordered_image = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )
    
    # Save image
    cv2.imwrite(output_file, bordered_image)
    
    print(f"\n✅ Marker saved: {output_file}")
    print(f"   Image size: {bordered_image.shape[1]}x{bordered_image.shape[0]} pixels")
    print("\n📐 PRINTING INSTRUCTIONS:")
    print("   1. Open the PNG file in an image viewer")
    print("   2. Print with these settings:")
    print("      - Paper size: A4 or Letter")
    print("      - Scale: 'Fit to page' or 'Actual size'")
    print("      - Orientation: Portrait")
    print("   3. Measure the printed marker:")
    print("      - BLACK SQUARE should be exactly 15cm x 15cm")
    print("      - Use a ruler to verify")
    print("   4. Mount on rigid surface (foam board, acrylic, cardboard)")
    print("   5. Ensure marker is flat (no warping)")
    print("\n⚠️  CRITICAL: Marker must be EXACTLY 15cm for accurate localization!")
    print("=" * 60)
    
    # Create a calibration guide image
    create_calibration_guide(marker_id, marker_size_px)

def create_calibration_guide(marker_id, marker_size_px):
    """Create a printable calibration guide with ruler marks"""
    
    try:
        border = 100
        total_size = marker_size_px + 2 * border
        
        # Create white canvas (RGB for color operations)
        guide = np.ones((total_size, total_size, 3), dtype=np.uint8) * 255
        
        # Generate marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
        
        # Convert marker to 3-channel
        marker_rgb = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        
        # Place marker in center
        guide[border:border+marker_size_px, border:border+marker_size_px] = marker_rgb
        
        # Add ruler marks (every cm)
        black = (0, 0, 0)
        
        # Top ruler
        for i in range(16):  # 0-15 cm
            x = border + int(i * marker_size_px / 15)
            cv2.line(guide, (x, border-30), (x, border-10), black, 2)
            cv2.putText(guide, str(i), (x-10, border-35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, black, 1)
        
        # Left ruler
        for i in range(16):
            y = border + int(i * marker_size_px / 15)
            cv2.line(guide, (border-30, y), (border-10, y), black, 2)
            cv2.putText(guide, str(i), (border-60, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, black, 1)
        
        # Add title
        cv2.putText(guide, "ArUco Marker ID=0 (15cm x 15cm)", 
                    (total_size//2 - 200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, black, 2)
        
        # Add instructions
        instructions = [
            "Print this page at 100% scale (no resizing)",
            "Verify: ruler marks should match actual cm",
            "Black square must be exactly 15cm x 15cm",
            "Mount on rigid, flat surface"
        ]
        
        y_offset = total_size - 80
        for i, text in enumerate(instructions):
            cv2.putText(guide, text, (50, y_offset + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, black, 1)
        
        # Save calibration guide
        cv2.imwrite('aruco_marker_id0_15cm_with_ruler.png', guide)
        print(f"✅ Calibration guide saved: aruco_marker_id0_15cm_with_ruler.png")
        print("   (Includes ruler marks for verification)")
    except Exception as e:
        print(f"⚠️  Warning: Could not create calibration guide: {e}")
        print("   (Main marker image is still available)")

def verify_opencv():
    """Verify OpenCV installation and ArUco module"""
    print("\nVerifying OpenCV installation...")
    
    try:
        cv_version = cv2.__version__
        print(f"✅ OpenCV version: {cv_version}")
        
        # Check if ArUco module is available
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        print("✅ ArUco module available")
        
        return True
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nPlease install OpenCV with ArUco support:")
        print("   pip install opencv-contrib-python")
        return False

if __name__ == '__main__':
    if not verify_opencv():
        sys.exit(1)
    
    generate_aruco_marker()
    
    print("\n📄 Files created:")
    print("   1. aruco_marker_id0_15cm.png - Simple marker")
    print("   2. aruco_marker_id0_15cm_with_ruler.png - With calibration ruler")
    print("\nNext steps:")
    print("   1. Print one of the marker images")
    print("   2. Verify size with a ruler (15cm x 15cm)")
    print("   3. Mount on rigid surface")
    print("   4. Place in operating room visible to camera + AVP")

