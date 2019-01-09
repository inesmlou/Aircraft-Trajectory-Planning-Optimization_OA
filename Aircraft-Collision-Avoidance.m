clear all
% close all
clc

T = 100;                           % tempo máximo de chegada
d = 1;                             % distância de segurança ao obstáculo
E = 0.001;                         % compromisso entre parcelas a minimizar
N = 3;                             % nº de veículos
%m( ) =[   ];                      % massa dos vários veículos kg
v_max = 0.8;                       % velocidade máxima, m/s
f_max = 0.3;                       % força máxima, N
%wmax = ;                          % velocidade angular para as curvas, º/s

%inicializações:
%inicializações:
x_i = [0;4;5]; y_i = [2;4;2];          % posição inicial
v_x_i = [0.3;0.3;0.3]; v_y_i = [0;0;0];% velocidade inicial
f_x_i = [0;0;0]; f_y_i = [0;0;0];      % força inicial
x_f = [10;8;10]; y_f = [5;2;3];        % posição final
% o_x = 6; o_y = 3;                    % posição do obstáculo             
% 
% o = [o_x o_y];

% criação de um vector coluna com os valores finais da posição para
% permitir a subtração do vector com a posição em cada iteração

for n=1:N
    p_final{n} = repmat([x_f(n) y_f(n)],T,1);
end

% início da optimização do problema:
P=[];
V=[];
F=[];

% ciclo para minimizar a trajectória de cada um dos aviões
for n=1:N
    cvx_begin
        variables p(T,2) v(T,2) f(T,2)           % variáveis da minimização
        minimize( norm( p - p_final{n}, 1 ) + E*(norm( f(:,1) , 1 ) + norm( f(:,2), 1 )) )
        subject to
            p(1,:) == [x_i(n) y_i(n)]
            v(1,:) == [v_x_i(n) v_y_i(n)]
            f(1,:) == [f_x_i(n) f_y_i(n)]
            for k = 1:T-1
                p(k+1,:) == p(k,:) + v(k,:)
                v(k+1,:) == v(k,:) + f(k,:)

                abs(f(k,1)) <= f_max
                abs(f(k,2)) <= f_max
                abs(v(k,1)) <= v_max
                abs(v(k,2)) <= v_max
            end
    cvx_end
    
    P{n} = p;
    V{n} = v;
    F{n} = f;
    
end

% Trajectórias dos aviões, em cada instante
figure (1)
hold on

    for n=1:N
      hold on
        plot( P{n}(:,1),P{n}(:,2),'o-','Color',[0.0 (n/N) 1.0]) % trajectória inicial do avião n
    end


for a = 2:N                         % correção da trajectória do avião 'a'
    
    for n = 1:a-1 
        I = 0;                                    % verificação da existência de colisão entre o avião 'a' e o avião 'n'
        for k = 1:T-1                             % Ciclo para a verificação do
            if( norm( P{a}(k,:)-P{n}(k,:)) < d)   % cumprimento da distância de -------------------------------------
                I = k;                            % segurança.
                break                             % Guarda-se na variável I o 
            end                                   % instante k da 1ª violação.....
        end
        % Caso I permaneça com o valor 0, significa que em nenhum instante se
        % violou a distância de segurança entre o avião 'a' e o avião 'n'

        % Ciclo de correcção da trajectória do avião 'a': 
        % Apenas entra neste ciclo caso ainda haja algum instante em que viole a
        % distância de segurança entre o avião 'a' e o 'n':
        while I ~= 0
            cvx_begin
                variables p_new(T,2) v_new(T,2) f_new(T,2)  % variavéis da minimização
                minimize( norm(p_new-p_final{a}, 1 )+ norm( p_new - P{a}, 1 ) + E*(norm( f_new(:,1) , 1 ) + norm( f_new(:,2), 1 )) )
                subject to
                    p_new(1,:) == [x_i(a) y_i(a)]
                    v_new(1,:) == [v_x_i(a) v_y_i(a)]
                    f_new(1,:) == [f_x_i(a) f_y_i(a)]

                    for k = 1:T-1
                        p_new(k+1,:) == p_new(k,:) + v_new(k,:)*(k+1-k)
                        v_new(k+1,:) == v_new(k,:) + f_new(k,:)*(k+1-k)

                        abs(f_new(k,1)) <= f_max
                        abs(f_new(k,2)) <= f_max
                        abs(v_new(k,1)) <= v_max
                        abs(v_new(k,2)) <= v_max
                    end

                    % certificar que a distância de segurança no primeiro ponto de violação, da trajectória anterior, é cumprida
                    ((p_new(I,:) - P{n}(I,:))*((P{a}(I,:)- P{n}(I,:))' / (norm(P{a}(I,:)-P{n}(I,:))))) >= d 
            cvx_end

            P{a} = p_new;
            V{a} = v_new;
            F{a} = f_new;
            
            % Verificação de existem mais conflitos/colisões do aviaçao 'a'
            % com o avião 'n':
            I=0;
            for k = 1: T-1                            % Ciclo para a verificação do
                if( norm( P{a}(k,:)-P{n}(k,:)) < d) % cumprimento da distância de -------------------------------------
                    I = k;                            % segurança.
                    break                             % Guarda-se na variável I o 
                end                                   % instante k da 1ª violação.....
            end
            
            hold on
        %     plot( p_new(:,1),p_new(:,2),'o-', 'color', [1.0 0.4 0.4]) % trajectória nova
%             for t=1:T
%                 for n=1:N
%                    plot( P{a}(t,1),P{a}(t,2),'o-','Color',[1.0 ((n-0.8)/N) 0.0]) % trajectória inicial do avião n
%                 end
%                 pause
%             end
            
            clear p_new v_new f_new
        end
    end   
end


