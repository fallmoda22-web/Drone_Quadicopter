%% ==========================================================
%  EVALUATION DE LA ROBUSTESSE - SIMULINK
% ===========================================================

clc;

%% Références

x_ref = 0;
y_ref = 0;
z_ref = 1;

%% Début de l'évaluation (après décollage)

t_eval_start = 5;

%% Seuils de robustesse

seuil_x = 0.15;
seuil_y = 0.15;
seuil_z = 0.05;

%% Données exportées depuis Simulink

t  = t_sim;
x  = x_sim;
y  = y_sim;
z  = z_sim;

% Nombre de rafales
nb_rafales = nb_rafales_sim;

%% Conversion Timeseries -> Tableau

if isa(t,'timeseries'),  t = t.Time;  end
if isa(x,'timeseries'),  x = x.Data;  end
if isa(y,'timeseries'),  y = y.Data;  end
if isa(z,'timeseries'),  z = z.Data;  end
if isa(nb_rafales,'timeseries')
    nb_rafales = nb_rafales.Data;
end

%% Mise en colonnes

t = t(:);
x = x(:);
y = y(:);
z = z(:);

nb_rafales = nb_rafales(end);

%% Durée de simulation

duree_simulation = t(end);

%% Ignorer la phase de décollage

idx = t >= t_eval_start;

%% Calcul des erreurs

ex = x_ref - x(idx);
ey = y_ref - y(idx);
ez = z_ref - z(idx);

max_ex = max(abs(ex));
max_ey = max(abs(ey));
max_ez = max(abs(ez));

%% Altitude

z_min = min(z(idx));
z_max = max(z(idx));

%% Vérification des critères

ok_x = max_ex <= seuil_x;
ok_y = max_ey <= seuil_y;
ok_z = max_ez <= seuil_z;

%% Affichage

fprintf('\n');
fprintf('=====================================================\n');
fprintf('        BILAN DE ROBUSTESSE - SIMULINK\n');
fprintf('=====================================================\n');

fprintf('Durée de simulation : %.1f s\n', duree_simulation);
fprintf('Evaluation à partir de : %.1f s\n', t_eval_start);
fprintf('Nombre de rafales : %d\n\n', nb_rafales);

fprintf('Erreur maximale en X : %.3f m\n', max_ex);
fprintf('Erreur maximale en Y : %.3f m\n', max_ey);
fprintf('Erreur maximale en Z : %.3f m\n\n', max_ez);

fprintf('Altitude minimale : %.3f m\n', z_min);
fprintf('Altitude maximale : %.3f m\n\n', z_max);

fprintf('Critere X (< %.2f m) : %s\n', seuil_x, string(ok_x));
fprintf('Critere Y (< %.2f m) : %s\n', seuil_y, string(ok_y));
fprintf('Critere Z (< %.2f m) : %s\n\n', seuil_z, string(ok_z));

if ok_x && ok_y && ok_z
    fprintf('STATUT : ROBUSTE\n');
else
    fprintf('STATUT : STABLE MAIS A AMELIORER\n');
end

fprintf('=====================================================\n');