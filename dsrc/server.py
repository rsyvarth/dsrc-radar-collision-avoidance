# Includ our local python packages
from sys import path
path.append('packages')
path.append('packages/python')

from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello World!\n'

if __name__ == '__main__':
    app.run('0.0.0.0')
