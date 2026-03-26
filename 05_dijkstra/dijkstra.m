function [coste, ruta] = dijkstra(G, origen, destino)
    n = size(G, 1);
    
    % 1. Inicialización
    nodo = Inf(n, 1); % f(n) coste del camino más corto desde el origen
    padre = zeros(n, 1); % Para recordar de dónde venimos y trazar la ruta
    visitado = false(n, 1); % Vector lógico para marcar los nodos ya evaluados
    
    % El coste para ir del origen al origen es siempre 0
    nodo(origen) = 0;
    
    % 2. Bucle principal de exploración
    for i = 1:n
        % Encontrar el nodo no visitado con el coste mínimo acumulado
        min_coste = Inf;
        u = -1;
        for j = 1:n
            if ~visitado(j) && nodo(j) < min_coste
                min_coste = nodo(j);
                u = j;
            end
        end
        
        % Si no encontramos más nodos alcanzables o ya llegamos al destino, paramos
        if u == -1 || u == destino
            break;
        end
        
        % Marcamos el nodo actual 'u' como evaluado
        visitado(u) = true;
        
        % Evaluar los vecinos del nodo 'u'
        for v = 1:n
            peso = G(u, v);
            % Si hay una conexión (peso > 0) y el vecino 'v' no está visitado
            if peso > 0 && ~visitado(v)
                nuevo_coste = nodo(u) + peso;
                % Si este nuevo camino es mejor que el que conocíamos, lo actualizamos
                if nuevo_coste < nodo(v)
                    nodo(v) = nuevo_coste;
                    padre(v) = u; % Guardamos que para llegar a 'v' lo mejor es pasar por 'u'
                end
            end
        end
    end 
    
    % 3. Reconstrucción de la ruta y asignación del coste final
    coste = nodo(destino);
    ruta = [];
    
    % Si el destino fue alcanzado (el coste no es Infinito)
    if coste ~= Inf
        actual = destino;
        while actual ~= 0
            ruta = [actual, ruta]; % Añadimos el nodo al principio del vector
            actual = padre(actual); % Retrocedemos al padre
        end
    end
end