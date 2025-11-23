from flask import Flask, request, render_template, jsonify, Response
import json
import os
import logging
import requests
import signal
import subprocess
from datetime import datetime
from flask_cors import CORS #由于 Flask 后端和 Vue 前端可能会在不同的端口上运行
#（比如 Flask 在 5000 端口，Vue 在 8080 端口），会遇到 跨域请求 问题，CORS解决跨域问题。
import time
import hashlib

# 日志配置
#flask前端日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("server.log"),
        logging.StreamHandler()
    ]
)

app = Flask(__name__, template_folder='.')
#xty:
#filepath='/home/user/桌面/new_INFANTRY/AutoAim_infantry'
#template_folder='filepath'

CORS(app)
def get_local_ip():
    """
    获取本机 IP 地址，要求格式为 192.168.137.*
    如果存在多个符合条件的地址，则返回第一个符合的地址；否则返回 None
    """
    import socket
    # 第一种方式：从 gethostbyname_ex 获取所有地址
    try:
        hostname = socket.gethostname()
        ips = socket.gethostbyname_ex(hostname)[2]
        for ip in ips:
            if ip.startswith("192.168.137."):
                return ip
    except Exception as e:
        print(f"获取IP（方式1）失败: {e}")

    # 第二种方式：通过和外部地址建立UDP套接字连接获取本机IP
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip.startswith("192.168.137."):
            return ip
    except Exception as e:
        print(f"获取IP（方式2）失败: {e}")

    # 如果没有符合条件的IP，则返回 None
    return None

# 动态配置

CONFIG_PATH = os.getenv('CONFIG_PATH', '../../config.json')
local_ip = get_local_ip()
print("find ip",local_ip)
logging.info(f"find ip:{local_ip}")
VIDEO_SOURCE_DEFAULT = f"http://{local_ip}:8081/stream" if local_ip else "http://192.168.137.184:8081/stream"
VIDEO_SOURCE_DEFAULT = "http://127.0.0.1:8081/stream"
VIDEO_SOURCE = os.getenv('VIDEO_SOURCE', VIDEO_SOURCE_DEFAULT)

def load_config():
    """加载配置文件（自动创建缺失文件）"""
    try:
        if not os.path.exists(CONFIG_PATH):
            os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
            with open(CONFIG_PATH, 'w') as f:
                json.dump({"initialized": False}, f)
            logging.info(f"创建默认配置文件: {CONFIG_PATH}")
        
        with open(CONFIG_PATH, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error(f"配置操作失败: {str(e)}")
        return {"error": str(e)}

def save_config(data):
    """保存配置文件"""
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        return True
    except Exception as e:
        logging.error(f"保存失败: {str(e)}")
        return False


def kill_aim():
    """杀掉所有aim相关进程"""
    try:
        # use command: ps -ef | grep autoaim | grep -v grep | awk '{print $2}'
        pids = subprocess.check_output(
            "ps -ef | grep Au | grep -v grep | awk '{print $2}'", 
            shell=True
        ).decode().strip().split()


        #xty:
        '''
        threads=subprocess.check_output(
            "ps -ef | grep thread | grep Au | awk '{print $2}'",
            shell=True
        ).decode().strip().split()
        ##csv格式的pids  #top 后面一个为一行pids
        for thread in threads:
            if thread not in threads:
                pids.append(thread)
        '''        


        for pid in pids:
            subprocess.run(['sudo', 'kill', pid])
        return True
    except Exception as e:
        logging.error(f"杀掉aim进程失败: {str(e)}")
        logging.error(f"可能需要sudo visudo配置免密码权限，添加：hustlyrm ALL=(ALL) NOPASSWD: /bin/kill")
        return False
    
#flask 注册的前端页面127.0.0。1：8081/stream
'''index() (Flask 路由 '/')

作用：渲染 editor.html 模板并注入 server_time 变量（当前时间字符串）。
返回：HTML 页面响应。
要点：该 HTML 负责前端 UI，通常会使用 /video_feed 来显示视频。'''
@app.route('/')
def index():
    return render_template('editor.html', 
                         server_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

'''将上游返回的分片数据以原样分片（iter_content）逐块转发给浏览器，
响应的 mimetype 为 multipart/x-mixed-replace，使浏览器能直接显示 MJPEG 实时画面。'''
@app.route('/api/config', methods=['GET'])
def get_config():
    config_data = load_config()
    if "error" in config_data:
        return jsonify(config_data), 500
    return jsonify(config_data)

@app.route('/api/config', methods=['POST'])
def update_config():
    try:
        new_config = request.get_json()
        if not new_config:
            return jsonify({"error": "空请求体"}), 400
        
        if save_config(new_config):
            kill_aim()
            '''用：接收前端提交的 JSON 配置并保存（save_config），
            保存成功后调用 kill_aim() 清理旧进程'''
            return jsonify({"status": "success"})
        #flask.__init__().jsonify

        return jsonify({"error": "保存失败"}), 500
    except Exception as e:
        logging.error(f"服务器错误: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/video_feed')
def video_feed():
    def generate():
        try:
            res = requests.get(VIDEO_SOURCE, stream=True, timeout=10)
            if res.status_code == 200:
                for chunk in res.iter_content(chunk_size=1024 * 512):
                    yield chunk #another thread yield to flask Response
                    #把字节传入浏览器。
            else:
                logging.warning(f"视频源异常: {res.status_code}")
        except Exception as e:
            logging.error(f"视频流错误: {str(e)}")
    #用：接收前端提交的 JSON 配置并保存（save_config），保存成功后调用 kill_aim() 清理旧进程
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame',#mjpeg格式。
        headers={'Cache-Control': 'no-cache'}
    )
if __name__ == '__main__':
    config = load_config()
    # 先获取 car_name 对应的值
    car_name = config.get("car_name")
    if car_name:
        # 使用 car_name 值获取下一级 JSON 部分
        param = config.get(car_name, {})
        web_debug = param.get("web_debug", True)
    else:
        web_debug = True
    
    if web_debug is False:
        logging.info("配置中 web_debug 为 false，终止 server 启动")
        exit(0)

    app.run(
        host=os.getenv('FLASK_HOST', '0.0.0.0'),
        port=int(os.getenv('FLASK_PORT', 8080)),
        debug=False,
        threaded=True,
        use_reloader=False
    )