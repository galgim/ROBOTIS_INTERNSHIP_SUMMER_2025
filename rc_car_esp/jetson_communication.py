def wrap_jetson_message(message):
    """Wraps a message in a start and end character and returns it.

    Start and end character can be any arbitrary characters, but must 
    be changed on both the RC car ESP as well as the Jetson codebase 
    """

    start_character = '%'
    end_character = '^'
    return start_character + message + end_character

def validate_jetson_message(message):
    """Checks if a message has a specified start and end character and
    if it is the specified size.

    Start and end character can be any arbitrary characters, but must 
    be changed on both the RC car ESP as well as the Jetson codebase.
    Same applies to the size.
    """
    bool_message_has_start_and_end = message.startswith('%') and message.endswith('^') #start and end character check
    bool_message_is_n_bytes = len(message) == 8 #message size check
    return bool_message_has_start_and_end and bool_message_is_n_bytes # returns true if it has start&end char, and 8 bytes
    