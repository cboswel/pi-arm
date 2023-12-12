#!/usr/bin/python3

import socket, sys, os, time

def send(ip, port, sock, message):
    sock.sendto(message.encode('utf-8'), (ip, port)) 
    print(message)

def main(message=0):

    port = 5432     #arbitrary port
    ip = "127.0.0.1"
    freq = 1
    period = (1 / freq)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        send(ip, port, sock, message)
        time.sleep(period)

if __name__ == "__main__":
    if len(sys.argv) == 4:
        args = ""
        for arg in range(1, 4):
            args += str(sys.argv[arg]) + " "
        main(args)
    else:
        feedback = "Usage is: ./sendUDP <X-coord> <Y-coord> <Z-coord>. Please try again with three arguments!"
        print(feedback)
