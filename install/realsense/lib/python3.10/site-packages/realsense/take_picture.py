import pyrealsense2 as rs
import numpy as np
import cv2
import os

# 設定儲存照片的資料夾
save_dir = "picture"
os.makedirs(save_dir, exist_ok=True)  # <--- 加這行就能避免錯誤
# 初始化 RealSense 相機
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 啟動攝影機
pipeline.start(config)

print("按下 [空白鍵] 拍照，按 [q] 離開")

try:
    while True:
        # 取得影像幀
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 轉換為 NumPy 陣列
        color_image = np.asanyarray(color_frame.get_data())

        # 顯示影像
        cv2.imshow("RealSense Camera", color_image)

        # 監聽鍵盤事件
        key = cv2.waitKey(1) & 0xFF

        # 按下空白鍵 (Space) 拍照
        if key == 32:  # 空白鍵的 ASCII 碼是 32
            image_path = os.path.join(save_dir, f"image_{len(os.listdir(save_dir)) + 1}.jpg")
            cv2.imwrite(image_path, color_image)
            print(f"已保存圖片: {image_path}")

        # 按下 'q' 退出程式
        elif key == ord("q"):
            break

finally:
    # 停止攝影機
    pipeline.stop()
    cv2.destroyAllWindows()