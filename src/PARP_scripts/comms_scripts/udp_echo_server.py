import socket
import time
import struct

HOST = "0.0.0.0"
PORT = 5005

# Client request:  !I Q   = (seq:uint32, t1_epoch_ns:uint64)
REQ_FMT = "!IQ"
# Server reply:    !I Q Q Q = (seq, t1, t2_server_rx, t3_server_tx)
REP_FMT = "!IQQQ"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"UDP Echo Server (timestamp) listening on {HOST}:{PORT}")

while True:
    data, addr = sock.recvfrom(2048)

    # Default fallback: echo
    resp = data

    try:
        if len(data) >= struct.calcsize(REQ_FMT):
            seq, t1 = struct.unpack(REQ_FMT, data[:struct.calcsize(REQ_FMT)])
            t2 = time.time_ns()          # server receive timestamp (epoch ns)
            t3 = time.time_ns()          # server send timestamp (epoch ns)
            resp = struct.pack(REP_FMT, seq, t1, t2, t3)
    except Exception:
        resp = data

    sock.sendto(resp, addr)