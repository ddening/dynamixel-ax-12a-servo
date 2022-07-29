import functools

def decorator_error_handling(function):
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except:
            print("!!! Error occured !!!")
            print("!!! Check .log file for further information !!!")
    return wrapper