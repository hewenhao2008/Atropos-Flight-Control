import urllib2
import time
def query(command):
     print command
     r=urllib2.Request('http://192.168.250.110/cgi-bin/ajaxinput?'+command)
     try:
         urllib2.urlopen(r)
     except Exception, err:
         return 1
     return 0

msg = []
pipe = open('/dev/input/js0', 'r')
encendido=0
guinnada_factor=5000
alabeo_factor=3000
cabeceo_factor=3000
gas=0
gas_start=60000000
gas_step=2000000
while 1:
    for char in pipe.read(1):
        msg += [ord(char)]
        if len(msg) == 8:
            if msg[6] == 1:
                
                if msg[4]  == 1:  #down key
                    if msg[7]==5:
                        encendido=0
                        print "Apagar"
                        query('Y0')

                    if msg[7]==4:
                        if encendido==0:
                            encendido=1
                            print "Encender"
                            query('z0');
                            time.sleep(1)
                            query('Y1000000')
                            print "Listo"
                            gas=gas_start
                            time.sleep(1)
                            query('L'+str(gas))
                    
                    if msg[7]==6:
                        if encendido==1:   
                            gas=gas+gas_step
                            query('L'+str(gas))
                    if msg[7]==7:
                        if encendido==1:
                            gas=gas-gas_step
                            query('L'+str(gas))                    
                            
            elif msg[6] == 2:
                if msg[5] > 127:
                    analog=255-msg[5]
                    analog=-analog
                else:
                    analog=msg[5]

                if msg[7]==0:#guinnada
                    if encendido==1:
                        guinnada=analog*guinnada_factor
                        query('G'+str(guinnada))
                
                if msg[7]==3:#cabeceo
                    if encendido==1:
                        cabeceo=analog*cabeceo_factor
                        query('A'+str(cabeceo))
                
                if msg[7]==2:#alabeo
                    if encendido==1:
                        alabeo=analog*alabeo_factor
                        query('C'+str(alabeo))
  
            msg = []
