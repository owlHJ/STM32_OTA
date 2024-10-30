from flask import Flask

app = Flask(__name__)

@app.route('/', methods=['GET'])
def home():
    return "Welcome to the Flask server!"

@app.route('/test', methods=['GET'])
def test():
    return "Hello, ESP8266!"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
