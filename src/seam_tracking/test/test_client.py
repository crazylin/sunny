import socket

def modbus_msg(id: str, valid: bool, u: float, v: float):
    id = int(id) % 0x10000
    # Transaction identifier
    s = id.to_bytes(2, 'big')
    # Protocol identifier 2, Length field 2, Unit identifier 1, Function code 1
    s += bytes([0x00, 0x00, 0x00, 0x0d, 0x01, 0x10])
    # Start address 2, number of registers, number of bytes
    s += bytes([0x00, 0x02, 0x00, 0x03, 0x06])

    b = bytes([0x00, 0xff]) if valid else bytes([0x00, 0x00])

    try:
        t = bytes()
        t += round(u * 100).to_bytes(2, 'big')
        t += round(v * 100).to_bytes(2, 'big')
    except Exception:
        s += bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    else:
        s += b + t

    return s

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect_ex(("127.0.0.1", 2345))
    s = modbus_msg("123", True, 0.01, 0.02)
    sock.sendall(s)

if __name__ == '__main__':
    main()
