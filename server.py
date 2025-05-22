import socket
import threading

HOST = '192.168.110.110'
PORTS = [20000, 40000]

def handle_client(client_socket, addr, client_id):
    print(f"[{client_id}] 클라이언트 연결됨: {addr}")
    try:
        while True:
            msg = input(f"[{client_id}] 보낼 색상(red, white, blue 등) 입력 (종료: exit): ").strip()
            if msg.lower() == "exit":
                print(f"[{client_id}] 서버를 종료합니다.")
                break
            if not msg:
                continue  # 빈 입력 무시

            client_socket.sendall(msg.encode('utf-8'))
            print(f"[{client_id}] 전송 완료: {msg}")

    except Exception as e:
        print(f"[{client_id}] 에러 발생: {e}")

    finally:
        client_socket.close()
        print(f"[{client_id}] 클라이언트 소켓 종료.")

def server_thread(port, client_id):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, port))
    server_socket.listen(1)
    print(f"[{client_id}] 서버가 {HOST}:{port}에서 대기 중입니다...")

    client_socket, addr = server_socket.accept()
    handle_client(client_socket, addr, client_id)
    server_socket.close()
    print(f"[{client_id}] 서버 소켓 종료.")

def main():
    threads = []
    for idx, port in enumerate(PORTS):
        t = threading.Thread(target=server_thread, args=(port, f"PORT{port}"))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

if __name__ == "__main__":
    main()
