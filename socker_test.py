import socket
import time

if __name__ == '__main__':
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8888))
    server.listen(0)
    connection, address = server.accept()
    print(connection, address)
    num = 0
    while True:
        # connection, address = server.accept()
        # print(connection, address)

        recv_str = connection.recv(1024)[0:5]
        print("enter 1")
        recv_str = recv_str.decode("ascii")
        if not recv_str:
            break
        num = num + 1
        print(recv_str, num)

        connection.send(bytes("clientRecv: %s," % recv_str, encoding="ascii"))
        time.sleep(0.5)

    connection.close()
    input("enter end")