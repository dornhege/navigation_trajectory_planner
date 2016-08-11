#!/usr/bin/env python

import yaml
import sys
import matplotlib.pyplot as plt

def time_to_first(run_stats):
    if not run_stats:
        return 20.0
    return run_stats[0]['time']
time_to_first.min = -0.5

def final_eps_reached(run_stats):
    if not run_stats:
        return 6.0
    eps = run_stats[-1]['eps']
    if eps > 6:
        eps = 6
    return eps
final_eps_reached.min = 0.9

def final_cost(run_stats):
    if not run_stats:
        return 100*1000
    cost = run_stats[-1]['cost']
    if cost > 100*1000:
        cost = 100*1000
    return cost
final_cost.min = 0

def expands(run_stats):
    if not run_stats:
        return 80*1000
    return sum([r['expands'] for r in run_stats])
expands.min = 0

def expands_to_first(run_stats):
    if not run_stats:
        return 9*1000
    return run_stats[0]['expands']
expands_to_first.min = 0

def scatter_pairs(data, param_values, fn):
    entries = []
    first = True
    for param_value in param_values:
        v = data[param_value]
        print "Processing", param_value
        for i, run in enumerate(v):
            if first:
                entries.append([fn(run["planner_stats"])])
            else:
                entries[i].append(fn(run["planner_stats"]))
        first = False
    return entries

def make_scatter_plot(ax, data, param_values, fn):
    print "make_scatter_plot for %s" % fn.__name__
    sp = scatter_pairs(data, param_values, fn)
    try:
        ax.set_xlim(fn.min, fn.max)
        ax.set_ylim(fn.min, fn.max)
    except AttributeError:
        pass
    ax.plot([e[0] for e in sp], [e[1] for e in sp], 'o', alpha=0.5)
    maxdata = max(max(e[0], e[1]) for e in sp)
    ax.plot([0,maxdata], [0, maxdata], '-', color='k', linewidth=2)
    labels = param_values
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(fn.__name__)

def make_time_histogram(ax, data, fn):
    print "make_time_histogram for %s" % fn.__name__
    try:
        ax.set_xlim(fn.min, fn.max)
        ax.set_ylim(0.0, 1.05)
    except AttributeError:
        pass
    data_time_points = {}
    for k, v in data.iteritems():
        print "Processing", k
        for run in v:
            time_point = fn(run["planner_stats"])
            if not k in data_time_points:
                data_time_points[k] = [time_point]
            else:
                data_time_points[k].append(time_point)
    for k, v in data_time_points.iteritems():
        v.sort()
    def make_hist(times):
        sum_acc = 0.0
        hist = {}
        hist[0.0] = 0.0
        for t in times:
            sum_acc += 1.0
            hist[t] = sum_acc
        hist = dict((k, v/sum_acc) for k,v in hist.iteritems())
        return hist
    for k in sorted(data_time_points.keys()):
        h = make_hist(data_time_points[k])
        points = [(k, v) for k,v in sorted(h.iteritems())]
        ax.plot([e[0] for e in points], [e[1] for e in points], '-')
    ax.plot([0, 10], [0.95, 0.95], '-', color='k')
    ax.legend(sorted(data_time_points.keys()), 'lower right')
    ax.set_title(fn.__name__)

def main():
    if len(sys.argv) < 3:
        print "Usage: %s <data.yaml> parameter_name [param_value1 param_value2]" % sys.argv[0]
        return 1

    pq_name = sys.argv[1]
    param_name = sys.argv[2]

    with open(pq_name, 'r') as f:
        all_data = yaml.load(f)
    data = all_data[param_name]

    param_values = data.keys()
    if len(param_values) != 2:
        param_values = sys.argv[3:5]
        assert len(param_values) == 2
        assert param_values[0] in data.keys()
        assert param_values[1] in data.keys()
    assert len(param_values) == 2, "Data had more than two values and choices not given in args"

    # check each param value has same number of queries
    query_entries_len = -1
    for param_value in param_values:
        query_entries = data[param_value]
        if query_entries_len < 0:
            query_entries_len = len(query_entries)
        assert len(query_entries) == query_entries_len


    f, axx = plt.subplots(3, 2)

    make_scatter_plot(axx[0, 0], data, param_values, time_to_first)
    make_scatter_plot(axx[0, 1], data, param_values, final_eps_reached)
    make_scatter_plot(axx[1, 0], data, param_values, final_cost)
    make_time_histogram(axx[1, 1], data, time_to_first)
    make_scatter_plot(axx[2, 0], data, param_values, expands_to_first)
    make_scatter_plot(axx[2, 1], data, param_values, expands)

    plt.show()

if __name__=="__main__":
    main()

