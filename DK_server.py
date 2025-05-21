import socket

HOST = '127.0.0.1'  # 서버 IP 주소
PORT = 20000        # 포트 번호

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"서버가 {HOST}:{PORT}에서 대기 중입니다...")

    client_socket, addr = server_socket.accept()
    print(f"클라이언트 연결됨: {addr}")

    try:
        while True:
            msg = input("클라이언트로 보낼 색상(red, white, blue 등) 입력 (종료: exit): ").strip()
            if msg.lower() == "exit":
                print("서버를 종료합니다.")
                break
            if not msg:
                continue  # 빈 입력 무시

            client_socket.sendall(msg.encode('utf-8'))
            print(f"전송 완료: {msg}")

    except Exception as e:
        print(f"에러 발생: {e}")

    finally:
        client_socket.close()
        server_socket.close()
        print("서버 소켓 종료.")

if __name__ == "__main__":
    main()
