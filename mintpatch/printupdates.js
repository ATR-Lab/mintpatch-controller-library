var python_script = 'MINTpatch_update.py';
const {PythonShell} = require('python-shell');
var pyshell= new PythonShell(python_script);

pyshell.on('message', function(message){
    console.log(message)
});

pyshell.end();