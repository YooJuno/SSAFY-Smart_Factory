# -*- coding: utf-8 -*-
import socket
import time
import threading

HOST = '127.0.0.1'
PORT = 20000

def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST,PORT))
            print('connected to {1}:{2}'.format(HOST, PORT))
            return s
        except socket.error as e:
            print('연결 실패')
            s.close()
            time.sleep(5)

try:
    while True:
        s = connect_to_server()

        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break

                cmd = data.decode()
                if cmd == '1':
                    print('cmd 1')

                elif cmd == '2':
                    print('cmd 2')
                
                elif cmd == '3':
                    print('cmd 3')
                
                elif cmd == '4':
                    print('cmd 4')

                else:
                    print('Invalid cmd')
                
                time.sleep(0.1)

            except socket.error as e:
                print('socket 통신 에러')
        
        s.close()

except KeyboardInterrupt:
    print('프로그램')
except Exception as e:
    print('error occured')
finally:
    print('끝') 
