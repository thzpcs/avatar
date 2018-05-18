# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import socket
from time import sleep


TCP_IP = '127.0.0.1'
TCP_PORT = 7003
BUFFER_SIZE = 8192
MESSAGE = "thing"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

def main():


    #s.send(MESSAGE)
    while True:
        s.send(MESSAGE)
        data = s.recv(BUFFER_SIZE)
        #packet = data.split('Char00')
        packet = data.split('\n')
        
        thing = packet[0]
        thing2 = packet[1]
        
        
        motionData = thing.split(' ')
        motionData2 = thing2.split(' ')
        # Index 97:103 - RightHand
        # Index 235:241 - LeftHand
        print motionData2#[160:176]
        print '\n'
        
        sleep(0.1)

if __name__ == "__main__":
    main()