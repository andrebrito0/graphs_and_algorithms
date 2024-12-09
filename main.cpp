#include "mat1.h"


int main()
{
  try {
        grafoNaoOri G(6);
        G.novaAresta(1,2,1);
        G.novaAresta(3,4,1);
        G.novaAresta(5,6,1);
        G.novaAresta(2,3,1);
        G.novaAresta(4,5,1);

        std:: cout<<"Imprimir grafo G"<<'\n';
        G.print();
        std::cout << '\n';
        std:: cout<<"Imprimir componentes conexas do grafo G"<<'\n';
        componentes X;
        X=G.componentes_conexas();
        X.print();
        std::cout << '\n';

        std:: cout<<"Imprimir DIJKSTRA do grafo g"<<'\n';
        path y;
        y = G.dijkstra(1,4);
        y.print();

        std:: cout<<"Imprimir DIJKSTRA Total do grafo g"<<'\n';
        path_t z;
        z = G.dijkstra(1);
        z.print();

        std::cout << '\n';
        std::cout << '\n';

        std::cout << "imprimir o grafo T: \n";
        grafoNaoOri T("/Users/AndreBrito/Downloads/grafoNaoOri/grafo.txt");
        T.print();



    return 0;
  }catch(const errGrafo& e) {
    std::cerr << "Excepção: " << e.what() << '\n';
    return 1;
  }catch(const std::exception& e) {
    std::cerr << "Excepção: " << e.what() << '\n';
    return 2;
  }catch(...) {
    std::cerr << "Excepção não prevista\n";
    return 3;
  }

}
