# Example program to receive packets from the radio link
#
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from lib_nrf24 import NRF24
import time 
import spidev

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]] 

radio = NRF24(GPIO, spidev.SpiDev()) 
radio.begin(0, 17) 

radio.setPayloadSize(32) 
radio.setChannel(0x76)    #thiết lập kênh 
radio.setDataRate(NRF24.BR_1MBPS)  #Tốc độ truyền
radio.setPALevel(NRF24.PA_MIN)   # Dung lượng truyền MIn

radio.setAutoAck(True) 
radio.enableDynamicPayloads() 
radio.enableAckPayload()

radio.openWritingPipe(pipes[0])     # Mở kênh phát
radio.openReadingPipe(1, pipes[1])  # Mở  kênh thu
radio.printDetails()   # In ra thông số Thu phát

radio.startListening()   #Bắt đầu thu  
while True:
    tinnhan = list("Chao Arduino")
    while len(tinnhan) <32:
        tinnhan.append(0)
    radio.stopListening()    # Dừng thu để Phát
    radio.write(tinnhan)
    radio.startListening()   #Bắt đầu thu  
    if radio.available(0):   # Nếu có Tín hiệu  tới
        receive = []           
        radio.read(receive, radio.getDynamicPayloadSize())
        print ("Chao Tu: {}".format(receive))       # In thông số nhận
        string = ""
        for n in receive:
            if (n>=32 and n<=126):
                string+= chr(n)
        print("Tin nhan: {}".format(string))    # In tin nhận được từ arduino
