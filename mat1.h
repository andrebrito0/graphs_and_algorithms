#ifndef MAT1_H
#define MAT1_H

#include <vector>
#include <list>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <stdexcept>
#include <iostream>
#include <functional>
#include <limits>
#include <sstream>
#include <fstream>
#include <set>


//Tipo de dados do identificador dos nodos
//TIPO_ID deve ser definido como:
// 1 - para usar int
// 2 - para usar char
// 3 - para usar string
#define TIPO_INT 1
#define TIPO_CHAR 2
#define TIPO_STRING 3

//Alterar aqui para usar outro tipo
#define TIPO_ID TIPO_INT

#if (TIPO_ID == TIPO_INT)
typedef int TipoId;
#elif (TIPO_ID == TIPO_CHAR)
typedef char TipoId;
#elif (TIPO_ID == TIPO_STRING)
typedef std::string TipoId;
#else
#error Tem que definir TIPO_ID como TIPO_CHAR, TIPO_INT ou TIPO_STRING
#endif

class errGrafo: public std::runtime_error
{
public:
    errGrafo(const std::string s):std::runtime_error(s) {}
};


class Arco
{
private:
    TipoId ori, dest; //identificadores dos nodos de origem e destino
    double peso;
public:
    Arco(const TipoId& o, const TipoId& d, double c):ori(o),dest(d),peso(c) {}
    const TipoId& u() const {return ori;}
    const TipoId& v() const {return dest;}
    double w() const {return peso;}
    void setW(double p) {peso=p;}
};

class Nodo
{
private:
    TipoId ident;
    std::vector<Arco> adjcs;
public:
    explicit Nodo(const TipoId& id) : ident(id) {}
    void setId(const TipoId& id) {ident=id;}
    const TipoId& id() const {return ident;}
    void insereArco(const Arco& a) {adjcs.push_back(a);}
    const std::vector<Arco>& adjs() const {return adjcs;}
    void apagaArco(const TipoId&);
};

struct componentes{

    std:: vector<std::vector<TipoId>> c;

    void print();
};

struct path{

    double cost; //custo do caminho
    std::vector<TipoId> caminho; //caminho
    TipoId source; //origem
    TipoId term; //terminal

    typedef std:: pair<double, std::pair<TipoId,TipoId>> trio; //(chave, (pred id, nodo id))
    std::vector<trio> p; //vetor com os custos calculados para os nodos e respetivos precedentes

    void print();
};

struct path_t{
    typedef std:: pair<double, std::pair<TipoId,TipoId>> trio; //(chave, (pred id, nodo id))
    std::vector<trio> p; //vetor com os custos calculados para os nodos e respetivos precedentes

    void print();

};

class grafoNaoOri
{
private:
    std::vector<Nodo> nodos;
    std::map<TipoId,unsigned int> id2idx; //mapeia identificadores em indíces do vector
public:
    explicit grafoNaoOri(size_t n) {
            //Cria um grafo com n nodos e atribui-lhes identificadores do tipo TipoId
            //Identificadores 1,2,...,n se o identificadoe for inteiro, A,B,.. se for char, N1,N2,.. se for string
    #if (TIPO_ID == TIPO_INT)
            nodos.resize(n,Nodo(0));
            for(size_t i=0;i<n;++i) {id2idx[i+1]=i; nodos[i].setId(i+1);}
    #elif (TIPO_ID == TIPO_CHAR)
            if(n>26) throw{"grafoNaoOri: identificadores do tipo char só são suportados até 26 nodos"};
            nodos.resize(n,Nodo(''));
            for(size_t i=0;i<n;++i) {id2idx['A'+i]=i; nodos[i].setId('A'+i);}
    #elif (TIPO_ID == TIPO_STRING)
            nodos.resize(n,Nodo(""));
            for(size_t i=0;i<n;++i) {id2idx["N"+std::to_string(i+1)]=i; nodos[i].setId("N"+std::to_string(i+1));}
    #endif
        }

    grafoNaoOri() {}
    grafoNaoOri(const std::string&);
    void novaAresta(const TipoId& ori, const TipoId& dest, double w=0);
    void novoNodo(const TipoId&);
    void apagaNodo(const TipoId&);
    void apagaAresta(const TipoId&, const TipoId&);
    void print() const;
    void DFS(const TipoId&) const;
    void DFS_r(unsigned int, std::vector<bool>&) const;
    void DFS_nr(const TipoId&) const;
    grafoNaoOri prim();
    void arestas_por_ordem();
    grafoNaoOri kruskal() const;
    componentes componentes_conexas() const;
    path dijkstra(const TipoId &, const TipoId &) const;
    path_t dijkstra(const TipoId &) const;



};

#endif
