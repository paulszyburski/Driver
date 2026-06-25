from time import sleep

def scan_with_ultrasonic(bot):
    print("test")
    distances = []




    bot.doMove(100, -100)
    sleep(2.17)
        
        #distance = bot.requestUltrasonicSensor(1, 4)
        #distances.append(distance)

        #print("distance:", distance)


    bot.doMove(0, 0)
