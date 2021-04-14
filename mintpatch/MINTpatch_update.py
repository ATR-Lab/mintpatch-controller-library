import zerorpc

if __name__=="__main__":
    printer=zerorpc.Client()
    printer.connect("tcp://127.0.0.2:4242")
    while True:
        print(printer.running_update(1,1))
    #print(printer.running_update(1,1))