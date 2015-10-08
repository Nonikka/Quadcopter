#-*- coding:utf8 -*-
import pygame
from trans import *
QUAD_IP = '192.168.137.100'
QUAD_PORT = 8099
import serial

try:
    serial_com3 = serial.Serial('com3',115200)
except:
    print("serial connect failed.")
# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

    
# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def printt(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
    

pygame.init()
 
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("joy")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()
    
# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while done==False:
    
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
            
 
    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.printt(screen, "Number of joysticks: {}".format(joystick_count) )
    textPrint.indent()
    
    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
    
        textPrint.printt(screen, "Joystick {}".format(i) )
        textPrint.indent()
    
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.printt(screen, "Joystick name: {}".format(name) )
        
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.printt(screen, "Number of axes: {}".format(axes) )
        textPrint.indent()
        _axis = [0,0,0,0,0,0]
        for i in range( axes ):
            axis = joystick.get_axis( i )
            textPrint.printt(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
            _axis[i] = int(axis * 100)#乘100利于传输
        textPrint.unindent()
        #_axis[3] 为上下，_axis[4]为左右
        #_axis[1] 控制油门
        try:
            n = serial_com3.write("U" + str("%04d" %-_axis[1]) + str("%04d" %-_axis[3]) + str("%04d" %_axis[4])+ str("%01d" %_buttons[4]) + str("%01d" %_buttons[5]) + "1111")
        except:
            print("serial send failed")
            
        buttons = joystick.get_numbuttons()
        textPrint.printt(screen, "Number of buttons: {}".format(buttons) )
        textPrint.indent()
        _buttons = [0,0,0,0,0,0,0,0,0,0]
        for i in range( buttons ):
            button = joystick.get_button( i )
            _buttons[i] = int(button)
            textPrint.printt(screen, "Button {:>2} value: {}".format(i,button) )
        textPrint.unindent()
            
        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        hats = joystick.get_numhats()
        textPrint.printt(screen, "Number of hats: {}".format(hats) )
        textPrint.indent()

        for i in range( hats ):
            hat = joystick.get_hat( i )
            textPrint.printt(screen, "Hat {} value: {}".format(i, str(hat)) )
        textPrint.unindent()
        
        textPrint.unindent()

    
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second
    clock.tick(20)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
#pygame.quit ()













