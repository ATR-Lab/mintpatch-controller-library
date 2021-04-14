var python_script = 'startup.py';
const {PythonShell} = require('python-shell');
var pyshell= new PythonShell(python_script);

pyshell.on('message', function(message){
    console.log(message)
});

//pyshell.end();
var python_script2 = 'MINTlisten.py';
var pyshell2= new PythonShell(python_script2);

pyshell2.send(JSON.stringify("scan"))
pyshell2.send(JSON.stringify("move port_1_001 3"))
pyshell2.send(JSON.stringify("end"))

pyshell2.on('message', function(message){
    console.log(message)
});

pyshell2.end()

//Whatever code you want between them
/*
var python_script3 = 'MINTpatch_update.py';
var pyshell3= new PythonShell(python_script3);

pyshell3.on('message', function(message){
    console.log(message)
});

pyshell3.end();
*/