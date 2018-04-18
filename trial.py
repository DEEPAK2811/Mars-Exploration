import serial
import bge
 def main():
ser=serial.Serial("COM5",115200)
x=ser.readline()
print(x)
cont = bge.logic.getCurrentController()
    own = cont.owner

    sens = cont.sensors['mySensor']
    actu = cont.actuators['myActuator']


if x==obj1 :
   movement1=[0.0,10.0,0.0]
  own.applymovement(movement1,False)

if x==ob2 :
   movement2=[0.0,-10.0,0.0] 
  own.applymovement(movement2,Flase)

main()



// function to send data
void uart_transmit (unsigned char data)
{
    while (!( UCSRA & (1<<UDRE)));                // wait while register is free
    UDR = data;                                   // load data in the register
}