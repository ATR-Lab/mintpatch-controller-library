import zerorpc
import sys
import json


def read_from_json():
    """
    Boiler-plate code for interacting with the commands sent from Node.js
    Returns a string
    """
    # jin becomes the next line in the JSON file created by the Node.
    # It needs to be translated
    jin = sys.stdin.readline()
    print(json.loads(jin))

    # loads() does the translation
    return json.loads(jin)

def end_listening(console_input, input_length):
    #Just returns false, causing the listener to exit its loop
    # and the program to end.
    return False


def ignore(console_input, input_length):
    #Used to prevent errors in the switch.
    print("into ignore")
    return True

def listen():
    """
    This is the function that MintPatch will be in while idle.
    Some Output is handled here.
    However, the main purpose of this function is to parse input and
    call the appropriate functions to handle the command cases.
    """

    listener=zerorpc.Client()
    listener.connect("tcp://127.0.0.1:4242")

    # This dictionary will be used as a switch statement
    # to quickly parse input.
    # It translates strings into the appropriate function,
    # Each of which has the same parameters and return.
    switch_dict={
        'move': listener.move_motor,
        'scan': listener.scan,
        'update': listener.running_update,
        'end' : end_listening
    }


    # LOOP SETUP
    Continue=True

    # Establishes a common, potentially infinite while loop.
    # It can be ended by inputing "end".
    while(Continue):
        
        # GATHERING INPUT

        # NODE.JS
        # Uses this input function to gather one line from the Node.
        # This is a string. 
        
        conin = read_from_json()        
        
        # CONSOLE
        #conin = sys.stdin.readline()

        # INPUT SETUP
        # Since it's a string, and the parts of the input should be seperated
        # by spaces; we split this to make it an array.
        
        cinar = conin.split()
        
        # Knowing the length of this string array will
        # let us catch errors and quickly skip faulty commands.
        l = cinar.__len__()

        # In this case, there is no input.
        # There's no reason to go through the loop.
        if l == 0:
            continue


        # SWITCH

        # This is the magical switch statement. It runs the first
        # word of the input through the switch dictionary,
        # finding the appropriate function to run. Then, it
        # passes the full input array and its length. Not every
        # function uses them, but this allows for a very efficient call.

        # In the case that the input is faulty, the switch will call the
        # "ignore" function. This does nothing.

        # Finally, each function returns a Boolean telling the listener
        # whether or not to continue the loop. Every function except "end"
        # returns true.
        Continue=switch_dict.get(cinar[0], ignore)(cinar,l)
        print(Continue)
        

    # END LOOP
    print("Thank you for using MintPatch!")
    listener.disconnect("tcp://127.0.0.1:4242")

if __name__ == '__main__':
    listen()