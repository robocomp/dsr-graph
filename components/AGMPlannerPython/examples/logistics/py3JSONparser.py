import sys, string
import json
sys.path.append('/usr/local/share/agm/')
from AGGL import *


def parsingJSON(file):
	print("Comienza el parseo del JSON")
	f = open(file)
	dsrmodel_dict = json.load(f)
	nodes = dict()
	listalinks=list()
	
	
	#print(dsrmodel_dict["DSRModel"]["symbols"])
	simbolos = dsrmodel_dict["DSRModel"]["symbols"]
	for symbol in simbolos:
		#COgemos las caracteristicas de cada elemento por su id
		elemento = simbolos[str(symbol)]
		id = elemento['id']
		x = elemento['attribute']['pos_x']['value']
		y = elemento['attribute']['pos_y']['value']
		type = elemento['type']
		#print(id,type, x, y)
		nodes[id]= AGMSymbol(id, type, [x, y])
		
		#Ahora necesitamos crear los enlaces
		print ('antes de links, el elemento es:',elemento)
		links = elemento['links']
		print ('Esto es e contenido de linnk',links)
		for link in links:
			dst = link['dst']
			label = link['label']
			src = link['src']
			#print (dst, src, label)
			listalinks.append(AGMLink(src, dst, label, enabled=True))
			
		
	grafo = AGMGraph(nodes, listalinks) 
	f.close()
	return grafo
	
#print(parsingJSON('mundodeprueba.json'))