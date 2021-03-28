# mintpatch-controller-library

To launch MintPatch, use "startup.py". It is designed to work with Node.js.
Alternative code is commented out and marked in the GUITranslator to make MintPatch work with the consule.
We will be passing strings into this program as various commands, whether by a node or by the consule.
The following are available commands in their format:

scan                          #provides a printout of every servo attached to the system.
end                           #ends the program
move <servo name> <velocity>  #moves the named servo at the provided velocity.   NOTE: Currently causes error with an invalid servo name.
stop                          #stops the movement of every servo attached to the system.
stop <servo name>             #stops the movement of one named servo.   NOTE: Currently causes error with an invalid servo name.
update <servo name>           #provides a printout of one named servo.  NOTE: Currently causes error with an invalid servo name.


---
Printouts of servos will be a string formatted as such:
<servo name> <state ID> <voltage> <temperature> <angle> <velocity>
