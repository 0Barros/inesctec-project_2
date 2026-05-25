from receiver import app

if __name__ == '__main__':
    print("Starting the telemetry to receive data ...")
    
    app.run(host='0.0.0.0', port=5001, debug=True)