from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route("/from_rpi", methods=["POST"])
def from_rpi():
    """
    Receives raw Android message forwarded by RPi.
    Flask keeps this endpoint alive and will handle
    every new message automatically.
    """
    data = request.get_json()
    print(f"[PC Algo Server] Got raw message from RPi: {data}")

    # TODO: run your algorithm here if needed
    # For now, just echo back a dummy response
    reply = {
        "ack": True,
        "processed": data.get("raw", "UNKNOWN"),
        "decision": "F"   # dummy action (Forward)
    }

    print(f"[PC Algo Server] Responding: {reply}")
    return jsonify(reply)

@app.route("/status", methods=["GET"])
def status():
    return jsonify({"status": "PC Algo Server online"})

if __name__ == "__main__":
    print("[PC Algo Server] Running on 0.0.0.0:5005 ...")
    app.run(host="0.0.0.0", port=5005)
