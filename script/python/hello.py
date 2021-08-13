#!/usr/bin/env
# -*- coding: utf-8 -*-

# hello.py

def application(environ, start_response):
    print environ
    start_response('200 OK', [('Content-Type', 'text/html')])
    return [b'<h1>Hello, web!</h1>']
