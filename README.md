# mintpatch-controller-library

To launch MintPatch, use "startup.py". It is designed to work with Node.js.
Alternative code is commented out and marked in the GUITranslator to make MintPatch work with the consule.
We will be passing strings into this program as various commands, whether by a node or by the consule.
The following are available commands in their format:

-scan                          
-end                           
-move [servo name] [velocity]                   
-stop                          
-stop [servo name]             
-update [servo name]


---
Printouts of servos will be a string formatted as such:         
[servo name] [state ID] [voltage] [temperature] [angle] [velocity]
