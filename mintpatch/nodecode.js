var python_script = 'startup.py';
const {PythonShell} = require('python-shell');
var pyshell= new PythonShell(python_script);

pyshell.send(JSON.stringify("scan"))

pyshell.on('message', function(message){
    console.log(message)
});

pyshell.end();