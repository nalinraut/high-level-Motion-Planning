def flatten(d):
    """Flattens the given dictionary / array of arrays into a single vector"""
    if isinstance(d,dict):
        return sum([flatten(v) for v in d.itervalues()],[])
    if hasattr(d,'__iter__'):
        return sum([flatten(v) for v in d],[])
    return [d]

def flatten_keys(d,name=''):
    """Returns the keys corresponding to the flattened entries of the given
    dictionary / array of arrays"""
    if isinstance(d,dict):
        return sum([flatten_keys(v,name+'['+k+']') for k,v in d.iteritems()],[])
    if hasattr(d,'__iter__'):
        return sum([flatten_keys(v,name+'[%d]'%(i,)) for i,v in enumerate(d)],[])
    return [name]
