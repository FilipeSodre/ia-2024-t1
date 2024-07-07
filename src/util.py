"""Funções auxiliares para o projeto"""

import math


# Heurística
def haversine(lat1, lon1, lat2, lon2):
    """Calcula a distância, em metros, entre duas coordenadas GPS."""
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # apply formulae
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    
    r = 6371  
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return r * c * 1000 


#
# Não altere este comentário e adicione suas funções ao final do arquivo.
#
