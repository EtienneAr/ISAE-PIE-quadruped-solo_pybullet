#!/usr/bin/env python3
"""
Very simple HTTP server in python for logging requests
Usage::
    ./server.py [<port>]
"""

# curl -d "param1=value1&param2=value2" -X POST http://localhost:80

from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import isae.runOptim as runOptim

index_Path = "isae/server/index.html"

class S(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        f = open(index_Path)
        self._set_response()
        self.wfile.write(f.read().encode('utf-8'))
        f.close()

    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = self.rfile.read(content_length) # <--- Gets the data itself
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                str(self.path), str(self.headers), post_data.decode('utf-8'))

        post_args = []
        for params in post_data.decode('utf-8').split('&'):
            post_args += [params.split('=')]

        
        try :
            dict_args = dict((arg[0], arg[1]) for arg in post_args)

            params = [dict_args["bodyHeight"],
                    dict_args["stepPeriod"],
                    dict_args["stepLen"],
                    dict_args["phasesOff_0"],
                    dict_args["phasesOff_1"],
                    dict_args["phasesOff_2"],
                    dict_args["phasesOff_3"],
                    dict_args["point0_x"],
                    dict_args["point0_y"],
                    dict_args["point1_x"],
                    dict_args["point1_y"],
                    dict_args["point2_x"],
                    dict_args["point2_y"],
                    dict_args["Kp"],
                    dict_args["Kd"],
                    ]
            res = runOptim.runSimu(params)
        except Exception:
            res = "Wrong arguments !"

        self._set_response()
        self.wfile.write(str(res).format(self.path).encode('utf-8'))

def run(server_class=HTTPServer, handler_class=S, port=8080, level=logging.CRITICAL):
    logging.basicConfig(level=level)
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    logging.info('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    logging.info('Stopping httpd...\n')

if __name__ == '__main__':
    from sys import argv

    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()