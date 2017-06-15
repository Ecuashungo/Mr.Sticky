import triangulation as tri
import LED as l
import math as m

blue = l.LED('blue')
red = l.LED('red')
yellow = l.LED('yellow')
green = l.LED('green')

#test with 3 angles
print('Test with 3 angles: ')
blue.angle = m.radians(154)
yellow.angle = m.radians(78)
green.angle = m.radians(275)
red.angle = None
triang = tri.Triangulation([blue, red, yellow, green])
pos = triang.run()
print('position: ', pos)
print('\n')

#test with 4 angles
print('Test with 4 angles: ')
blue.angle = m.radians(160)
yellow.angle = m.radians(89)
green.angle = m.radians(286)
red.angle = m.radians(22)
triang2 = tri.Triangulation([blue, red, yellow, green])
pos = triang2.run()
print('position: ', pos)
