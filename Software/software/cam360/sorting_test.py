import LED as l

blue = l.LED('blue')
blue.angle = 10
red = l.LED('red')
red.angle = 20
yellow = l.LED('yellow')
yellow.angle = 30

led_list = [blue, red, yellow]

newlist = sorted(led_list, key=lambda x: x.angle, reverse=True)
newlist2= sorted(led_list, key=lambda x: x.angle, reverse=False)

print(newlist)
print(newlist2)