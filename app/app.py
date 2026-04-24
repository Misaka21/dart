from flask import Flask, Response, render_template
import argparse
import bridge

app = Flask(__name__)

@app.route('/')
def index():
    # 获取视频名称集合并转换为列表，以便能够索引
    video_names = bridge.get_cvmat_names()
    # 将集合转换为列表
    video_names_list = list(video_names)
    # 如果有视频流，选择第一个，否则使用空字符串
    default_video = video_names_list[0] if video_names_list else ""
    return render_template("index.html", video_name=default_video)

@app.route('/video_feed/<name>')
def video_feed(name):
    return Response(bridge.get_cvmat_jpegcode(name), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", "-s", nargs='+', default=[], help="startup script(s) before start the web app")
    opt = parser.parse_args()
    for script in opt.script:
        print(f"running startup script: '{script}'")
        exec(open(script).read())
    app.run(host="0.0.0.0", port=3000, threaded=True)