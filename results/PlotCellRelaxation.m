 
clear; close all;

base_dir = '../../../output/PerMedCoE/';
run_suffixes = {'1','05','025','0125','00625','1_120'};
run_labels = {'dt = 1.0','dt = 0.5','dt = 0.25','dt = 0.125','dt = 0.0625','dt = 1/120'};
run_colours = lines(numel(run_suffixes));
initial_separation_um = NaN;
run_times = cell(numel(run_suffixes), 1);
run_separations = cell(numel(run_suffixes), 1);

hold on
for i = 1:numel(run_suffixes)
	run_dir = [base_dir,'TestCellRelaxation_dt_',run_suffixes{i},'/results_from_time_0/'];
	data_file = [run_dir,'node_locations.dat'];

	if exist(data_file,'file') ~= 2
		warning(['Missing file: ',data_file]);
		continue
	end

	position_data = load(data_file);
	run_times{i} = position_data(:,1);
	run_separations{i} = 10*(position_data(:,5)-position_data(:,2));
	if isnan(initial_separation_um)
		initial_separation_um = 10*(position_data(1,5)-position_data(1,2));
	end
	plot(run_times{i}, run_separations{i}, '-', 'LineWidth', 1.5, ...
		'Color', run_colours(i,:), 'DisplayName', run_labels{i})
end

if ~isnan(initial_separation_um)
	b = 10;
	a = -10;
	t_ode = linspace(0, 5, 400);
	x0 = initial_separation_um;
	ode_rhs = @(t, x) a*log(x/b);
	[ts, xs] = ode45(ode_rhs, t_ode, x0);
	plot(ts, xs, 'k--', 'LineWidth', 1.8, 'DisplayName', 'ODE: dx/dt = a log(x/b)')

	t_exact_csv = 0:(1.0/1200.0):5;
	[t_exact_csv, x_exact_csv] = ode45(ode_rhs, t_exact_csv, x0);
	exact_csv = 'ChasteForceCellRelaxationExactSolution_dt_1_1200.csv';
		fid = fopen(exact_csv, 'w');
		fprintf(fid, 'time,exact_solution\n');
		for r = 1:numel(t_exact_csv)
			fprintf(fid, '%.16g,%.16g\n', t_exact_csv(r), x_exact_csv(r));
		end
		fclose(fid);
		disp(['Wrote exact solution table: ', exact_csv]);
else
	warning('ODE solution not plotted because no dt data files were found.');
end

yline(10, 'k:', 'DisplayName', 'R_A + R_B')
axis([0,5,8.9,10.1])
legend('Location','best')

title("Chaste Force Cell Separation")
ylabel("Separation (um)")
xlabel("Time (min)")
SaveAsPngEpsAndFig(-1,'ChasteForceCellRelaxationSweep', 12, 7/5, 12)


% Error curves: compare each dt run against ODE solution at that run's own
% simulation timesteps (no interpolation of simulation data).
if ~isnan(initial_separation_um)
	figure
	hold on

	for i = 1:numel(run_suffixes)
		if isempty(run_times{i})
			continue
		end

		t_eval = run_times{i};
		x_sim = run_separations{i};

		[ts_eval, xs_eval] = ode45(ode_rhs, t_eval, initial_separation_um);
		error_vals = abs(x_sim - xs_eval);

		plot(ts_eval, error_vals, '-', 'LineWidth', 1.5, ...
			'Color', run_colours(i,:), 'DisplayName', run_labels{i})

		error_csv = ['ChasteForceCellRelaxationError_dt_', run_suffixes{i}, '.csv'];
		fid = fopen(error_csv, 'w');
		fprintf(fid, 'time,error\n');
		for r = 1:numel(ts_eval)
			fprintf(fid, '%.16g,%.16g\n', ts_eval(r), error_vals(r));
		end
		fclose(fid);
		disp(['Wrote error table: ', error_csv]);
	end

	title('Error vs Time Against ODE Solution')
	ylabel('|x_{sim} - x_{ode}| (um)')
	xlabel('Time (min)')
	legend('Location','best')
	SaveAsPngEpsAndFig(-1,'ChasteForceCellRelaxationErrorSweep', 12, 7/5, 12)
else
	warning('Error plot skipped because no dt data files were found.');
end


figure 

base_dir = '../../../output/PerMedCoE/';
run_suffixes = {'001','01','1'};
run_labels = {'drag = 0.01','drag = 0.1','drag = 1.0'};
run_colours = lines(numel(run_suffixes));

hold on
for i = 1:numel(run_suffixes)
	run_dir = [base_dir,'TestCellHertzRelaxation_drag_',run_suffixes{i},'/results_from_time_0/'];
	data_file = [run_dir,'node_locations.dat'];

	if exist(data_file,'file') ~= 2
		warning(['Missing file: ',data_file]);
		continue
	end

	position_data = load(data_file);
	plot(position_data(:,1)/60, (position_data(:,5)-position_data(:,2)), '-', 'LineWidth', 1.5, ...
		'Color', run_colours(i,:), 'DisplayName', run_labels{i})
end

yline(10^(-5), 'k:', 'DisplayName', 'R_A + R_B')
axis([0,180,8.9*10^(-6),10.1*10^(-6)])
legend('Location','best')





title("Hertz Cell Separation")
ylabel("Separation (m)")
xlabel("Time (min)")
SaveAsPngEpsAndFig(-1,'HertzCellRelaxationSweep', 12, 7/5, 12)


