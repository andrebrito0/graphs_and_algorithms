#include "mat1.h"


    void Nodo :: apagaArco(const TipoId& dest) {
        for(unsigned int i=0;i<adjcs.size();++i)
            if(adjcs[i].v()==dest) {
                adjcs.erase(adjcs.begin()+i);
                return;
            }
    }


    void componentes :: print(){
        for(size_t i=0;i<c.size();i++){
            std::cout<<"Componente "<<i+1<<": ";
            for(size_t j=0;j<c[i].size();j++){
                std::cout<<c[i][j]<<" ";
            }
            std::cout<<'\n';
        }
    }


    void path :: print(){
        for(size_t i=1;i<p.size();i++){
            std::cout<<"Predecessor: "<<p[i].second.first<<" Nodo: "<<p[i].second.second<<" Valor da marca: "<<p[i].first<<'\n';
        }
        std::cout<<'\n';
        std::cout<<"Caminho mais curto entre: "<<source<<" e "<<term<<": "<<'\n';


        size_t i=caminho.size();
        while(i!=0){
            if(i==1) std::cout<<caminho[i-1];
            else std::cout<<caminho[i-1]<<" -- ";
            i=i-1;
        }
        std::cout<<'\n';
        std::cout<<"O comprimento do caminho é de "<< cost;
        std::cout<<'\n';

    }

    void path_t :: print(){
        for(size_t i=1;i<p.size();i++){
            std::cout<<"Predecessor: "<<p[i].second.first<<" Nodo: "<<p[i].second.second<<" Valor da marca: "<<p[i].first<<'\n';
        }
        std::cout<<'\n';
    }




    grafoNaoOri::grafoNaoOri(const std::string& loc){
            std::ifstream ler;
            ler.open(loc);
            if (!ler){
                std::cerr << "problema na abertura do ficheiro" <<'\n';
                exit (1);
            }
            size_t V,E;
            char ch1;
            ler>>V>>ch1>>E;

            nodos.resize(V,Nodo(0));
            for(size_t i=0;i<V;++i) {id2idx[i+1]=i; nodos[i].setId(i+1);}

            std::string s;

            while(std::getline(ler,s)){

                TipoId u,v;
                double w;
                char ch2,ch3;
                size_t i=1;
                while(ler>>u>>ch2>>v>>ch3>>w && i<=E){
                    //Não suportamos "loops". ori e dest têm que ser diferentes
                    if(u==v)
                        throw errGrafo{"novaAresta: origem e destino têm que ser diferentes."};
                    //ori e dest são identificadores. Têm que existir
                    if(id2idx.find(u)==id2idx.end() || id2idx.find(v)==id2idx.end())
                        throw errGrafo{"novaAresta: nodo inválido"};
                    //Já existe?
                    for(const auto& arco:nodos[id2idx.at(u)].adjs())
                        if(arco.v()==v) throw errGrafo{"novaAresta: a aresta já existe."};
                    //Inserimos dois arcos opostos
                    nodos[id2idx[u]].insereArco(Arco(u,v,w));
                    nodos[id2idx[v]].insereArco(Arco(v,u,w));
                    i++;
                }

            }

        }

    void grafoNaoOri :: novaAresta(const TipoId& ori, const TipoId& dest, double w) {
        //Não suportamos "loops". ori e dest têm que ser diferentes
        if(ori==dest)
            throw errGrafo{"novaAresta: origem e destino têm que ser diferentes."};
        //ori e dest são identificadores. Têm que existir
        if(id2idx.find(ori)==id2idx.end() || id2idx.find(dest)==id2idx.end())
            throw errGrafo{"novaAresta: nodo inválido"};
        //Já existe?
        for(const auto& arco:nodos[id2idx.at(ori)].adjs())
            if(arco.v()==dest) throw errGrafo{"novaAresta: a aresta já existe."};
        //Inserimos dois arcos opostos
        nodos[id2idx[ori]].insereArco(Arco(ori,dest,w));
        nodos[id2idx[dest]].insereArco(Arco(dest,ori,w));
    }

    void grafoNaoOri :: novoNodo(const TipoId& id) {
        if(id2idx.find(id)!=id2idx.end())
            throw errGrafo{"novoNodo: identificador já existe"};
        nodos.push_back(Nodo(id));
        id2idx[id]=nodos.size()-1;
    }

    void grafoNaoOri :: apagaNodo(const TipoId& nodo) {
        //O grafo é não orientado. Em nodos[nodo] existe toda a informação sobre arcos
        //que terminam em nodo
        const std::vector<Arco>& adjs=nodos[id2idx.at(nodo)].adjs();
        for(size_t i=0;i<adjs.size();++i) //precorremos os adjacentes a nodo
            nodos[id2idx.at(adjs[i].v())].apagaArco(nodo); //removemos o arco inverso no nó de destino
        //Removemos de nodos
        nodos.erase(nodos.begin()+id2idx.at(nodo));
        //Refazemos id2idx
        id2idx.clear();
        for(unsigned int i=0;i<nodos.size();++i) id2idx[nodos[i].id()]=i;
    }

    void grafoNaoOri :: apagaAresta(const TipoId& ori, const TipoId& dest) {
        nodos[id2idx.at(ori)].apagaArco(dest);
        nodos[id2idx.at(dest)].apagaArco(ori);
    }

    void grafoNaoOri :: print() const {
        for(size_t i=0;i<nodos.size();++i) {
            for(const auto& arco:nodos[i].adjs())
                if(id2idx.at(arco.u())<id2idx.at(arco.v())) //O grafo não é orientado. Mostramos apenas arco num sentido
                    std::cout << arco.u() << " -- " << arco.v() << " : " <<
                                 arco.w() << '\n';
        }
    }

    void grafoNaoOri :: DFS(const TipoId& inicio) const{
        if(id2idx.find(inicio)==id2idx.end()){
            throw errGrafo{"DFS inicio invalido"};
        }
        std:: vector<bool> visitado(nodos.size(),false);
        DFS_r(id2idx.at(inicio),visitado);
        std::cout<< '\n';
        std::cout<< '\n';

    }

    void grafoNaoOri :: DFS_r(unsigned int u, std::vector<bool>& visitado) const{
        visitado[u]=true;
        std::cout<<nodos[u].id()<<' ';

        for(const auto& arco:nodos[u].adjs()){
            TipoId v=arco.v();
            unsigned int idxV=id2idx.at(v);
            if(!visitado[idxV]){
                DFS_r(idxV,visitado);
            }
        }
    }


    void grafoNaoOri :: DFS_nr(const TipoId& source) const{
        std::stack<unsigned int> s;
        std:: vector<bool> visitado(nodos.size(),false);
        unsigned int u;
        s.push(id2idx.at(source));
        while(!s.empty()){
            u=s.top();s.pop();
            if(visitado[u]==false){
                std::cout<<nodos[u].id()<<' ';
                visitado[u]=true;
                for(const auto& x: nodos[u].adjs()){
                    s.push(id2idx.at(x.v()));

                }
            }
        }
    }

    //typedef std:: pair<double, unsigned int> par; (custo,nodo)

    grafoNaoOri grafoNaoOri :: prim(){

        typedef std:: pair<double, unsigned int> par; //(custo, idx_nodo)
        std:: priority_queue<par, std:: vector<par>, std:: greater<par>> pq;
        std:: vector<double> chave(nodos.size(), std:: numeric_limits<double>::infinity());
        std:: vector<bool> naArvore(nodos.size(), false);
        std:: vector<int> pai(nodos.size(),-1);
        pq.push(std::make_pair(0.0,0));

        while(!pq.empty()){
            unsigned int idx_u=pq.top().second;
            pq.pop();
            naArvore[idx_u]=true;
            for(const auto& arco: nodos[idx_u].adjs()){
                unsigned int idx_v=id2idx.at(arco.v());
                if(!naArvore[idx_v] && arco.w()<chave[idx_v]){
                    pai[idx_v]=idx_u;
                    chave[idx_v]=arco.w();
                    pq.push(std::make_pair(chave[idx_v],idx_v));
                }
          }
        }
        /*
        std::vector<Arco> arcosArvore; //Para guardar a árvore geradora mínima
        for(size_t i=1; i<pai.size();++i) {
            if(pai[i]==-1) //O grafo não é conexo
                    throw errGrafo{"prim: o grafo não é conexo."};
            arcosArvore.push_back(Arco(nodos[pai[i]].id(),nodos[i].id(),chave[i]));
        }
        */

        grafoNaoOri res(nodos.size());
        for(size_t i=1; i<pai.size();i++){
            if(pai[i]==-1) throw errGrafo{"prim: o grafo não é conexo."}; //O grafo não é conexo
            res.novaAresta(nodos[pai[i]].id(),nodos[i].id(),chave[i]);
        }
        return res;
    }

    void grafoNaoOri :: arestas_por_ordem(){
        typedef std:: pair <double, std::pair<TipoId,TipoId>> par;
        std:: priority_queue<par, std:: vector<par>,std:: greater<par>> pq;
        for(size_t i=0;i<nodos.size();++i) {
            for(const auto& arco:nodos[i].adjs())
                if(id2idx.at(arco.u())<id2idx.at(arco.v()))
                    pq.push(std::make_pair(arco.w(),std::make_pair(arco.u(),arco.v())));
        }
        while(!pq.empty()){
            double peso=pq.top().first;
            TipoId ori=pq.top().second.first;
            TipoId dest=pq.top().second.second;
            pq.pop();
            std::cout<<ori<<" -- "<<dest<<" : "<<peso<<'\n';
        }
    }


    grafoNaoOri grafoNaoOri :: kruskal() const{

        typedef std:: pair <double, std::pair<TipoId,TipoId>> par;
        std:: priority_queue<par, std:: vector<par>,std:: greater<par>> pq;
        std:: vector<TipoId> pai(nodos.size(),-1); //classicar componentes conexas
        std::vector<Arco> A_naArvore;
        std:: vector<bool> naArvore(nodos.size(), false);
        grafoNaoOri res(nodos.size());

        if(componentes_conexas().c.size()>1) throw errGrafo{"O Grafo não é conexo"};
        // lista prioritária para as aretas (custo, (ori,dest))
        for(size_t i=0;i<nodos.size();i++) {
            for(const auto& arco:nodos[i].adjs())
                if(id2idx.at(arco.u())<id2idx.at(arco.v()))
                    pq.push(std::make_pair(arco.w(),std::make_pair(arco.u(),arco.v())));
        }
        while(A_naArvore.size()<nodos.size()-1){
            double peso=pq.top().first;
            TipoId ori=pq.top().second.first;
            TipoId dest=pq.top().second.second;
            unsigned int idx_ori=id2idx.at(pq.top().second.first);
            unsigned int idx_dest=id2idx.at(pq.top().second.second);
            pq.pop();

            if(naArvore[idx_ori]==false && naArvore[idx_dest]==false){
                A_naArvore.push_back(Arco(ori,dest,peso));
                res.novaAresta(ori,dest,peso);
                pai[idx_ori]=ori;
                pai[idx_dest]=ori;
                naArvore[idx_ori]=true;
                naArvore[idx_dest]=true;

            }
            if(naArvore[idx_ori]==true && naArvore[idx_dest]==false){
                pai[idx_dest]=pai[idx_ori];
                naArvore[idx_dest]=true;
                A_naArvore.push_back(Arco(ori,dest,peso));
                res.novaAresta(ori,dest,peso);

            }
            if(naArvore[idx_ori]==true && naArvore[idx_dest]==true){
                if(pai[idx_dest]!=pai[idx_ori]){
                    A_naArvore.push_back(Arco(ori,dest,peso));
                    res.novaAresta(ori,dest,peso);
                    for(size_t i=0;i<pai.size();i++){
                        if(i!=idx_dest){
                            if(pai[i]==pai[idx_dest]) pai[i]=pai[idx_ori];
                        }
                    }
                    pai[idx_dest]=pai[idx_ori];
                }
            }
            if(naArvore[idx_ori]==false && naArvore[idx_dest]==true){
                pai[idx_ori]=pai[idx_dest];
                naArvore[idx_ori]=true;
                A_naArvore.push_back(Arco(ori,dest,peso));
                res.novaAresta(ori,dest,peso);
            }
        }

        //falta encontrar limite para o número de interações do ciclo while caso o grafo seja não conexo
        return res;

    }

    componentes grafoNaoOri :: componentes_conexas() const{
        componentes res;
        std::vector<bool> visitado(nodos.size(),false);
        //início
        for(size_t i=0;i<nodos.size();i++){
            if(visitado[i]==false){
                std::stack<unsigned int> s;
                s.push(i);
                std:: vector<TipoId> componente;
                unsigned int u;

                while(!s.empty()){
                    u=s.top();s.pop();
                    if(visitado[u]==false){
                        componente.push_back(nodos[u].id());
                        visitado[u]=true;
                        for(const auto& x: nodos[u].adjs()){
                            s.push(id2idx.at(x.v()));
                        }
                    }
                }
                res.c.push_back(componente);
            }
        }
        return res;
    }


     path grafoNaoOri :: dijkstra(const TipoId &source, const TipoId &term) const{
        path r;
        std:: vector<TipoId> pred(nodos.size(),-1);
        std:: vector<double> chave(nodos.size(), std:: numeric_limits<double>::infinity());
        std:: vector<bool> marca(nodos.size(),false);
        typedef std:: pair<double, TipoId> par; //(chave, nodo id)
        typedef std:: pair<double, std::pair<TipoId,TipoId>> trio; //(chave, (pred id, nodo id))
        std:: priority_queue<par, std:: vector<par>,std:: greater<par>> pq;
        std:: vector<trio> res;

        if(componentes_conexas().c.size()>1) throw errGrafo{"O Grafo não é conexo"};
        if(source==term) throw errGrafo{"Não exsitem loops"};
        if(id2idx.find(term)==id2idx.end() || id2idx.find(source)==id2idx.end()) throw errGrafo{"Não existem esses nodos"};

        //inicialização

        unsigned int idx_t=id2idx.at(term);
        unsigned int idx_s=id2idx.at(source);
        pq.push(std:: make_pair(0,source));
        chave[idx_s]=0;
        pred[idx_s]=0;

        while(!marca[idx_t]){

            par p = pq.top();
            unsigned int idx_n=id2idx.at(p.second);
            if(!marca[idx_n]) res.push_back(std::make_pair(chave[idx_n],std::make_pair(pred[idx_n],p.second)));
            marca[idx_n]=true;
            pq.pop();

            for(const auto& arco: nodos[idx_n].adjs()){
                unsigned int idx_v=id2idx.at(arco.v());
                if(!marca[idx_v] && (chave[idx_n]+arco.w())<chave[idx_v]){ //se não tiver marca permanente e d(i)+ C_ij<d(j)
                    pred[idx_v]=nodos[idx_n].id();
                    chave[idx_v]=chave[idx_n]+arco.w();
                    pq.push(std::make_pair(chave[idx_n]+arco.w(),arco.v())); //vai para a fila
                }
            }
        }

        TipoId a=term;
        std::vector<TipoId> v;
        v.push_back(a);
        while(a!=source){
            v.push_back(pred[id2idx.at(a)]);
            a=pred[id2idx.at(a)];
        }
        r.caminho = v;
        r.p = res;
        r.cost = chave[id2idx.at(term)];
        r.source = source;
        r.term = term;
        return r;

    }

     path_t grafoNaoOri :: dijkstra(const TipoId &source) const{
        path_t r;
        std:: vector<TipoId> pred(nodos.size(),-1);
        std:: vector<double> chave(nodos.size(), std:: numeric_limits<double>::infinity());
        std:: vector<bool> marca(nodos.size(),false);
        typedef std:: pair<double, TipoId> par; //(chave, nodo id)
        typedef std:: pair<double, std::pair<TipoId,TipoId>> trio; //(chave, (pred id, nodo id))
        std:: priority_queue<par, std:: vector<par>,std:: greater<par>> pq;
        std:: vector<trio> res;

        if(componentes_conexas().c.size()>1) throw errGrafo{"O Grafo não é conexo"};
        if(id2idx.find(source)==id2idx.end()) throw errGrafo{"Não existem esses nodos"};

        //inicialização

        unsigned int idx_s=id2idx.at(source);
        pq.push(std:: make_pair(0,source));
        chave[idx_s]=0;
        pred[idx_s]=0;
        while(!pq.empty()){

            par p = pq.top();
            unsigned int idx_n=id2idx.at(p.second);
            if(!marca[idx_n]) res.push_back(std::make_pair(chave[idx_n],std::make_pair(pred[idx_n],p.second)));
            marca[idx_n]=true;
            pq.pop();

            for(const auto& arco: nodos[idx_n].adjs()){
                unsigned int idx_v=id2idx.at(arco.v());
                if(!marca[idx_v] && (chave[idx_n]+arco.w())<chave[idx_v]){ //se não tiver marca permanente e d(i)+ C_ij<d(j)
                    pred[idx_v]=nodos[idx_n].id();
                    chave[idx_v]=chave[idx_n]+arco.w();
                    pq.push(std::make_pair(chave[idx_n]+arco.w(),arco.v())); //vai para a fila
                }
            }
        }

        r.p = res;

        return r;

    }




