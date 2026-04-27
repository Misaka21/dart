from flask import Flask, Response, render_template, request, stream_with_context
import argparse
import bridge
import json
import time

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

@app.route('/api/telemetry/stream')
def telemetry_stream():
    try:
        start_seq = int(request.args.get("after_seq", "0"))
    except ValueError:
        start_seq = 0

    @stream_with_context
    def generate():
        last_seq = start_seq
        while True:
            payload = bridge.poll_telemetry(last_seq)
            last_seq = int(payload.get("next_seq", last_seq))
            yield "data: " + json.dumps(payload, separators=(",", ":")) + "\n\n"
            time.sleep(bridge.get_telemetry_publish_interval_s())

    headers = {
        "Cache-Control": "no-cache",
        "X-Accel-Buffering": "no",
    }
    return Response(generate(), mimetype='text/event-stream', headers=headers)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", "-s", nargs='+', default=[], help="startup script(s) before start the web app")
    opt = parser.parse_args()
    for script in opt.script:
        print(f"running startup script: '{script}'")
        exec(open(script).read())
    app.run(host="0.0.0.0", port=3000, threaded=True)
