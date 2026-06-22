from time import sleep

def scan_with_ultrasonic(bot):
    print("test")
    distances = []
    for i in range(31):
        bot.doMove(80, -80)
        
        distance = bot.requestUltrasonicSensor(1, 4)
        distances.append(distance)

        print("distance:", distance)
        sleep(0.1)
    bot.doMove(0, 0)
