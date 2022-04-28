filename = 'cosine.csv';

data = csvread(filename);
N = 3;      % n carros

X = cell(N,1);

for k = 0:N-1
    idx = data(:,1) == k;
    X{k+1} = data(idx,2:end);
    
    Y = data(idx,2:end);
    plot(Y(:,1),Y(:,3), 'DisplayName',"Carro "+k, 'linewidth', 1), grid on, hold on
end
legend()
xlabel('Tiempo (ms)'), ylabel('Distancia (cm)'), title('Error')