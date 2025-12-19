# 匯入必要的函式庫
from flask import Flask, Response, render_template_string
import cv2
import threading
import time

# --- 全域變數設定 ---
# 建立一個線程鎖，用於安全地在不同線程間交換影像幀
lock = threading.Lock()
# 初始化 Flask 應用程式
app = Flask(__name__)
# 建立一個變數來存放攝影機物件
video_capture = None
# 建立一個變數來存放給網頁的最終影像幀
output_frame = None

def capture_frames():
    """
    這個函式會在一個獨立的背景線程中執行，專門負責從攝影機讀取畫面。
    """
    global video_capture, output_frame, lock
    
    # 不斷循環讀取
    while True:
        # 如果攝影機沒有開啟，就稍等一下再試
        if video_capture is None or not video_capture.isOpened():
            time.sleep(0.5)
            continue
            
        # 從攝影機讀取一幀畫面
        ret, frame = video_capture.read()
        
        # 如果成功讀取
        if ret:
            # --- 在這裡可以對影像進行任何處理 ---
            # 例如，加上時間戳
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, frame.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # ------------------------------------
            
            # 鎖定資源，將處理好的畫面更新到全域變數中
            with lock:
                output_frame = frame.copy()
        else:
            # 如果讀取失敗，可能是攝影機斷開了，等待一下
            print("無法讀取攝影機畫面...")
            time.sleep(0.5)

def generate_frame():
    """
    這是一個生成器函式，會被 Flask 用來產生影像串流。
    """
    global output_frame, lock
    
    # 不斷循環
    while True:
        # 鎖定資源，複製一份當前的影像幀
        with lock:
            if output_frame is None:
                # 如果還沒有畫面，就跳過這次循環
                continue
            # 將影像編碼為 JPEG 格式
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        
        # 以 multipart/x-mixed-replace 格式產出影像幀的 byte 資料
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')
        
        # 稍微等待，控制串流的幀率，減輕CPU負擔
        time.sleep(0.03)

@app.route("/")
def index():
    """
    網站的首頁，會顯示一個包含影像的 HTML 頁面。
    """
    # 使用 render_template_string 來處理 HTML 樣板
    html_template = """
    <html>
        <head>
            <title>羅技攝影機即時影像串流</title>
            <style>
                body { background-color: #333; color: white; font-family: sans-serif; text-align: center; margin: 0; padding: 0; }
                h1 { background-color: #555; padding: 1rem; }
                img { border: 2px solid #00caff; margin-top: 20px; width: 90%; max-width: 960px; height: auto; }
            </style>
        </head>
        <body>
            <h1>羅技攝影機即時影像串流</h1>
            <img src="{{ url_for('video_feed') }}">
        </body>
    </html>
    """
    return render_template_string(html_template)

@app.route("/video_feed")
def video_feed():
    """
    影像串流的路由。
    """
    # 回傳一個 Response 物件，內容是生成器函式產生的影像串流
    return Response(generate_frame(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# --- 主程式入口 ---
if __name__ == '__main__':
    try:
        # 初始化攝影機
        # 0 通常代表系統預設的第一個攝影機 (例如內建或第一個USB攝影機)
        # 如果你有好幾個攝影機，可以試著改成 1, 2, ...
        video_capture = cv2.VideoCapture(0)
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not video_capture.isOpened():
            raise RuntimeError("錯誤：無法開啟攝影機。請檢查是否已連接或被其他程式佔用。")

        # 啟動背景線程來抓取攝影機畫面
        capture_thread = threading.Thread(target=capture_frames)
        capture_thread.daemon = True
        capture_thread.start()
        
        # 啟動 Flask 網頁伺服器
        # host='0.0.0.0' 讓區域網路中的其他裝置可以連線
        print("網頁伺服器已啟動，請在瀏覽器中開啟 http://<你的Pi的IP地址>:8080")
        app.run(host='0.0.0.0', port=8080, threaded=True)

    except Exception as e:
        print(e)
    finally:
        # 程式結束時，釋放攝影機資源
        if video_capture is not None:
            video_capture.release()
            print("攝影機資源已釋放。")