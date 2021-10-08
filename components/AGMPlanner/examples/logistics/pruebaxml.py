import sys, string
import xml.etree.ElementTree as ET 
sys.path.append('/usr/local/share/agm/')
from AGGL import *



class AGMWorldModelpython3parser():
    def parsingxml(self,file):
        print('Comienzan mis pruebas')
        tree = ET.parse(file)
        self.nodes = dict()
        self.links=list()
        self.world = False
        self.currentSymbol = None
        print ('el tree es:',tree)
        root = tree.getroot()
        print ('el root es:',root)
        
        
        #Comprobamos la etiqueta inicial
        if root.tag.lower() != 'agmmodel':
            print ("<AGMModel> tag is missing!!")
            return 0
        #Si la etiqueta es correcta recorremos todo el arbol XML
        for child in root:
            #print (child.tag,child.attrib)
            
            #Si es un symbolo
            if child.tag == 'symbol':
                print('es un simbolo')
                id = child.attrib['id']
                type = child.attrib['type']
                x = child.attrib['x']
                y = child.attrib['y']
                print('id=',id,' type=',type,' x=',x,' y=',y)
                self.nodes[id]= AGMSymbol(id, type, [x, y])
                
            #Si es un enlace    
            if child.tag == 'link':
                print('es un enlace')
                src=child.attrib['src']
                dst=child.attrib['dst']
                label=child.attrib['label']
                print('src=',src,' dst=',dst,' label=',label)
                
                #Si el nodo origen o destino no existen hay error
                if not src in self.nodes:
                    print(('No src node', src))
                    sys.exit(-1)
                if not dst in self.nodes:
                    print(('No dst node', dst))
                    sys.exit(-1)
                    
                 #COmprobamos si el enlace esta desactivado en el XML    
                enabled = True
                try:
                    if child.attrib['enabled'].lower() in ["false", "f", "fa", "fal", "fals", "0", "n", "no"]:
                        enabled = False
                except:
                    pass

                self.links.append(AGMLink(src, dst, label, enabled=enabled))
                
        grafo = AGMGraph(self.nodes, self.links)      
        print(grafo)  
        return grafo
    
file='init0.xml'
g = AGMWorldModelpython3parser().parsingxml(file)