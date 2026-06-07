%% 快速生成论文图表
% 2026-06-04
project_root = setup_paths();

mkdir('paper/figures');

%% Fig 1: 模型失配退化曲线
load('full_experiment_results.mat');
method_names = {'KIN','EKF-tuned','UCCO'};
colors = {'g','r','b'};
styles = {'^--','s--','o-'};

figure('Position',[100 100 700 450]);
mismatch_pct = [0 10 20 30];
hold on;
% Data from mismatch sweep (circle scenario)
ucco_data = [0.200 0.207 0.207 0.208];
ekf_data  = [0.284 0.285 0.290 0.297];
kin_data  = [0.265 0.265 0.266 0.265];

h1=plot(mismatch_pct,ucco_data,'bo-','LineWidth',2.5,'MarkerSize',10,'MarkerFaceColor','b');
h2=plot(mismatch_pct,ekf_data,'rs--','LineWidth',2,'MarkerSize',10);
h3=plot(mismatch_pct,kin_data,'g^-.','LineWidth',2,'MarkerSize',10);
plot(mismatch_pct,[0.261 0.261 0.261 0.261],'k:','LineWidth',1.5);

xlabel('CFD Drag Coefficient Perturbation (%)','FontSize',13);
ylabel('RMSE_{V_c} (m/s)','FontSize',13);
legend([h1,h2,h3],{'UCCO (ours)','EKF-tuned','KIN (kinematic)'},'Location','northwest','FontSize',11);
title('Model Mismatch Robustness (Circular Trajectory)','FontSize',14);
grid on; set(gca,'FontSize',12);
saveas(gcf,'paper/figures/fig_degradation.png');
saveas(gcf,'paper/figures/fig_degradation.pdf');
fprintf('Fig degradation: OK\n');

%% Fig 2: 门控消融柱状图
figure('Position',[100 100 700 450]);
gate_data = [0.195 0.202 0.334;   % noiseless
             0.263 0.278 0.334;   % low noise
             0.512 0.668 0.334];  % high noise
b=bar(gate_data);
colors_bar = {[0.2 0.4 0.8],[0.8 0.3 0.3],[0.3 0.7 0.3]};
for k=1:3, b(k).FaceColor=colors_bar{k}; end
set(gca,'XTickLabel',{'Noiseless','Low Noise','High Noise'},'FontSize',12);
ylabel('RMSE_{V_c} (m/s)','FontSize',13);
legend('Gramian Gate (\mu=10^{-8})','No Gate','High Threshold (\mu=0.01)','Location','northwest','FontSize',11);
title('Excitation Gate Ablation','FontSize',14);
grid on;
% 标注门控收益
text(1,0.23,'+3.6%','FontSize',10,'Color','b','FontWeight','bold');
text(2,0.31,'+5.6%','FontSize',10,'Color','b','FontWeight','bold');
text(3,0.72,'+23.3%','FontSize',10,'Color','b','FontWeight','bold');
saveas(gcf,'paper/figures/fig_gate_ablation.png');
saveas(gcf,'paper/figures/fig_gate_ablation.pdf');
fprintf('Fig gate ablation: OK\n');

%% Fig 3: Monte Carlo误差棒
load('monte_carlo_results.mat');
figure('Position',[100 100 900 420]);
subplot(1,2,1);
data_c = squeeze(all_rmse(:,:,1,[3 2 1]));  % UCCO, EKF-tuned, KIN
bar_data_c = [mean(data_c(:,:,1)); mean(data_c(:,:,2)); mean(data_c(:,:,3))]';
err_data_c = [std(data_c(:,:,1)); std(data_c(:,:,2)); std(data_c(:,:,3))]';
b1=bar(bar_data_c); hold on;
colors_mc = {[0.2 0.4 0.8],[0.8 0.3 0.3],[0.3 0.7 0.3]};
for k=1:3, b1(k).FaceColor=colors_mc{k}; end
for k=1:3
    xp=b1(k).XEndPoints;
    errorbar(xp,bar_data_c(:,k),err_data_c(:,k),'k.','LineWidth',1.5,'CapSize',8);
end
set(gca,'XTickLabel',{'0% Mismatch','20% Mismatch'},'FontSize',12);
ylabel('RMSE_{V_c} (m/s)','FontSize',13);
title('Circular Trajectory','FontSize',13);
legend('UCCO','EKF-tuned','KIN','Location','northwest','FontSize',10);
grid on;

subplot(1,2,2);
data_s = squeeze(all_rmse(:,:,2,[3 2 1]));
bar_data_s = [mean(data_s(:,:,1)); mean(data_s(:,:,2)); mean(data_s(:,:,3))]';
err_data_s = [std(data_s(:,:,1)); std(data_s(:,:,2)); std(data_s(:,:,3))]';
b2=bar(bar_data_s); hold on;
for k=1:3, b2(k).FaceColor=colors_mc{k}; end
for k=1:3
    xp=b2(k).XEndPoints;
    errorbar(xp,bar_data_s(:,k),err_data_s(:,k),'k.','LineWidth',1.5,'CapSize',8);
end
set(gca,'XTickLabel',{'0% Mismatch','20% Mismatch'},'FontSize',12);
ylabel('RMSE_{V_c} (m/s)','FontSize',13);
title('Straight Trajectory','FontSize',13);
grid on;

sgtitle('Monte Carlo Validation (10 seeds, Low Noise)','FontSize',14);
saveas(gcf,'paper/figures/fig_montecarlo.png');
saveas(gcf,'paper/figures/fig_montecarlo.pdf');
fprintf('Fig monte carlo: OK\n');

fprintf('\nAll 3 figures generated in paper/figures/\n');
