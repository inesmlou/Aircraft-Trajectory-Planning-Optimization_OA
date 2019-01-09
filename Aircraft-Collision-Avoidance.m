clear all
% close all
clc

T = 100;                           % tempo m�ximo de chegada
d = 1;                             % dist�ncia de seguran�a ao obst�culo
E = 0.001;                         % compromisso entre parcelas a minimizar
N = 3;                             % n� de ve�culos
%m( ) =[   ];                      % massa dos v�rios ve�culos kg
v_max = 0.8;                       % velocidade m�xima, m/s
f_max = 0.3;                       % for�a m�xima, N
%wmax = ;                          % velocidade angular para as curvas, �/s

%inicializa��es:
%inicializa��es:
x_i = [0;4;5]; y_i = [2;4;2];          % posi��o inicial
v_x_i = [0.3;0.3;0.3]; v_y_i = [0;0;0];% velocidade inicial
f_x_i = [0;0;0]; f_y_i = [0;0;0];      % for�a inicial
x_f = [10;8;10]; y_f = [5;2;3];        % posi��o final
% o_x = 6; o_y = 3;                    % posi��o do obst�culo             
% 
% o = [o_x o_y];

% cria��o de um vector coluna com os valores finais da posi��o para
% permitir a subtra��o do vector com a posi��o em cada itera��o

for n=1:N
    p_final{n} = repmat([x_f(n) y_f(n)],T,1);
end

% in�cio da optimiza��o do problema:
P=[];
V=[];
F=[];

% ciclo para minimizar a traject�ria de cada um dos avi�es
for n=1:N
    cvx_begin
        variables p(T,2) v(T,2) f(T,2)           % vari�veis da minimiza��o
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

% Traject�rias dos avi�es, em cada instante
figure (1)
hold on

    for n=1:N
      hold on
        plot( P{n}(:,1),P{n}(:,2),'o-','Color',[0.0 (n/N) 1.0]) % traject�ria inicial do avi�o n
    end


for a = 2:N                         % corre��o da traject�ria do avi�o 'a'
    
    for n = 1:a-1 
        I = 0;                                    % verifica��o da exist�ncia de colis�o entre o avi�o 'a' e o avi�o 'n'
        for k = 1:T-1                             % Ciclo para a verifica��o do
            if( norm( P{a}(k,:)-P{n}(k,:)) < d)   % cumprimento da dist�ncia de -------------------------------------
                I = k;                            % seguran�a.
                break                             % Guarda-se na vari�vel I o 
            end                                   % instante k da 1� viola��o.....
        end
        % Caso I permane�a com o valor 0, significa que em nenhum instante se
        % violou a dist�ncia de seguran�a entre o avi�o 'a' e o avi�o 'n'

        % Ciclo de correc��o da traject�ria do avi�o 'a': 
        % Apenas entra neste ciclo caso ainda haja algum instante em que viole a
        % dist�ncia de seguran�a entre o avi�o 'a' e o 'n':
        while I ~= 0
            cvx_begin
                variables p_new(T,2) v_new(T,2) f_new(T,2)  % variav�is da minimiza��o
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

                    % certificar que a dist�ncia de seguran�a no primeiro ponto de viola��o, da traject�ria anterior, � cumprida
                    ((p_new(I,:) - P{n}(I,:))*((P{a}(I,:)- P{n}(I,:))' / (norm(P{a}(I,:)-P{n}(I,:))))) >= d 
            cvx_end

            P{a} = p_new;
            V{a} = v_new;
            F{a} = f_new;
            
            % Verifica��o de existem mais conflitos/colis�es do avia�ao 'a'
            % com o avi�o 'n':
            I=0;
            for k = 1: T-1                            % Ciclo para a verifica��o do
                if( norm( P{a}(k,:)-P{n}(k,:)) < d) % cumprimento da dist�ncia de -------------------------------------
                    I = k;                            % seguran�a.
                    break                             % Guarda-se na vari�vel I o 
                end                                   % instante k da 1� viola��o.....
            end
            
            hold on
        %     plot( p_new(:,1),p_new(:,2),'o-', 'color', [1.0 0.4 0.4]) % traject�ria nova
%             for t=1:T
%                 for n=1:N
%                    plot( P{a}(t,1),P{a}(t,2),'o-','Color',[1.0 ((n-0.8)/N) 0.0]) % traject�ria inicial do avi�o n
%                 end
%                 pause
%             end
            
            clear p_new v_new f_new
        end
    end   
end


