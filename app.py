import argparse
import logging

from flask import Flask, jsonify, render_template, request

from control_system import MotionController

app = Flask(__name__)
controller = None

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/move', methods=['POST'])
def move():
    data = request.json
    direction = data['direction']
    speed = float(data['speed'])
    return jsonify(controller.enqueue_move(direction, speed))


@app.route('/stop', methods=['POST'])
def stop():
    return jsonify(controller.enqueue_stop())


@app.route('/home', methods=['POST'])
def home():
    return jsonify(controller.enqueue_home())


@app.route('/cycle', methods=['POST'])
def cycle():
    data = request.json
    gap = float(data['gap'])
    speed = float(data['speed'])
    hold = float(data['hold'])
    cycles = int(data['cycles'])
    return jsonify(controller.enqueue_cycle(gap, speed, hold, cycles))


@app.route('/position', methods=['GET'])
def position():
    return jsonify(controller.get_position_payload())


def parse_args():
    parser = argparse.ArgumentParser(description='电机控制服务')
    parser.add_argument('--simulate', action='store_true', help='启用实时模拟模式（不连接真实电机/GPIO）')
    parser.add_argument('--port', type=int, default=49110, help='服务端口')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    controller = MotionController(simulate=args.simulate)

    try:
        controller.startup()
        mode = '模拟模式' if args.simulate else '真实硬件模式'
        controller.debug_log(f'服务启动成功（{mode}），监听 0.0.0.0:{args.port}')
        app.run(host='0.0.0.0', port=args.port, debug=False)
    except Exception as e:
        if controller:
            controller.debug_log(f'启动失败: {e}')
        else:
            print(f'启动失败: {e}')
    finally:
        if controller:
            controller.shutdown()
