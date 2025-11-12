#!/usr/bin/env python3
"""
Script per verificare compatibilità dipendenze qi_unipa_2
"""
import sys

def check_imports():
    """Verifica che tutti gli import funzionino"""
    errors = []
    
    # ROS2
    try:
        import rclpy
        print("✅ rclpy OK")
    except ImportError as e:
        errors.append(f"❌ rclpy: {e}")
    
    try:
        from cv_bridge import CvBridge
        print("✅ cv_bridge OK")
    except ImportError as e:
        errors.append(f"❌ cv_bridge: {e}")
    
    try:
        from sensor_msgs.msg import Image
        print("✅ sensor_msgs OK")
    except ImportError as e:
        errors.append(f"❌ sensor_msgs: {e}")
    
    try:
        from geometry_msgs.msg import Point, PointStamped
        print("✅ geometry_msgs OK")
    except ImportError as e:
        errors.append(f"❌ geometry_msgs: {e}")
    
    # Computer Vision
    try:
        import cv2
        import numpy as np
        print(f"✅ OpenCV {cv2.__version__} + NumPy {np.__version__} OK")
        
        # Test compatibilità
        test_array = np.zeros((100, 100, 3), dtype=np.uint8)
        _ = cv2.cvtColor(test_array, cv2.COLOR_BGR2GRAY)
        print("✅ OpenCV ↔ NumPy compatibility OK")
        
    except ImportError as e:
        errors.append(f"❌ cv2/numpy: {e}")
    except Exception as e:
        errors.append(f"❌ OpenCV-NumPy compatibility issue: {e}")
    
    # Web
    try:
        import webbrowser
        import http.server
        print("✅ Web modules OK")
    except ImportError as e:
        errors.append(f"❌ Web modules: {e}")
    
    # Pydantic (per LangChain tools)
    try:
        from pydantic import BaseModel, Field
        import pydantic
        print(f"✅ Pydantic {pydantic.__version__} OK")
    except ImportError as e:
        errors.append(f"❌ pydantic: {e}")
    
    return errors


def check_numpy_opencv_compatibility():
    """Verifica compatibilità NumPy con OpenCV e ROS2"""
    try:
        import numpy as np
        import cv2
        
        # Test dtype compatibility
        for dtype in [np.uint8, np.float32, np.float64]:
            arr = np.zeros((10, 10), dtype=dtype)
            _ = cv2.resize(arr, (20, 20))
        
        print("✅ NumPy dtypes compatibili con OpenCV")
        return True
        
    except Exception as e:
        print(f"❌ Incompatibilità NumPy-OpenCV: {e}")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("VERIFICA DIPENDENZE qi_unipa_2")
    print("=" * 60)
    print()
    
    errors = check_imports()
    
    print()
    print("=" * 60)
    check_numpy_opencv_compatibility()
    print("=" * 60)
    
    if errors:
        print("\n⚠️  ERRORI RILEVATI:")
        for err in errors:
            print(f"  {err}")
        sys.exit(1)
    else:
        print("\n✅ Tutte le dipendenze sono OK!")
        sys.exit(0)
