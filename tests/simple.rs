#[cfg(test)]
mod tests {
    use petgraph::graph::NodeIndex;
    use petgraph::stable_graph::EdgeReference;
    use generic_rcsp::{Label, ProblemGraph, UserProblem, RcspSolver, Meta, InitialLabelGenerator};


    #[derive(Debug,Clone)]
    struct LabelMeta {
        time : u8
    }
    impl Meta for LabelMeta {}
    impl InitialLabelGenerator<SimpleProblem> for LabelMeta {
        fn default(problem: &SimpleProblem) -> Self {
            LabelMeta { time : 0}
        }
    }

    #[derive(Debug,Clone)]
    struct NodeWeight{
        earliest_arrival: u8,
        latest_arrival: u8
    }
    struct  EdgeWeight{
        travel_time : u8,
        cost : u8
    }

    /// First example from
    /// Irnich S, Desaulniers G, 2005 Shortest Path Problems with Resource Constraints. Desaulniers G, Desrosiers
    // J, Solomon MM, eds., Column Generation, 33–65 (Boston, MA: Springer US), ISBN 978-0-387-25486-9.
    struct SimpleProblem {}

    impl UserProblem<LabelMeta, NodeWeight, EdgeWeight, ()> for SimpleProblem {
        fn create_graph(&mut self) -> ProblemGraph<NodeWeight, EdgeWeight> {

            let mut dag = petgraph::stable_graph::StableGraph::new();
            let s = dag.add_node( NodeWeight { earliest_arrival: 0, latest_arrival: 0});
            let n1 = dag.add_node(NodeWeight { earliest_arrival: 6, latest_arrival: 14});
            let n2 = dag.add_node(NodeWeight { earliest_arrival: 9, latest_arrival: 12});
            let n3= dag.add_node(NodeWeight { earliest_arrival: 8, latest_arrival: 12});
            let t = dag.add_node(NodeWeight { earliest_arrival: 9, latest_arrival: 15});

            dag.add_edge(s,n1, EdgeWeight { travel_time : 8, cost : 3});
            dag.add_edge(s,n2, EdgeWeight { travel_time : 5, cost : 5});
            dag.add_edge(s,n3, EdgeWeight { travel_time : 12, cost : 2});

            dag.add_edge(n1,t, EdgeWeight { travel_time : 4, cost : 7});
            dag.add_edge(n2,t, EdgeWeight { travel_time : 2, cost : 6});
            dag.add_edge(n3,t, EdgeWeight { travel_time : 4, cost : 3});

            return ProblemGraph{ start : s, end : t, dag}

        }

        fn is_dominating(label_a: &Label<LabelMeta>, label_b: &Label<LabelMeta>, active_filters: &[()]) -> bool {
            false
        }

        fn extend_label<'arena>(&self, existing_label: &'arena Label<'arena, LabelMeta>, edge: &EdgeReference<EdgeWeight>, source: &NodeWeight, target: &NodeWeight, active_filters: &[()], sink: NodeIndex) -> Option<Label<'arena, LabelMeta>> {

            let arrival_time = (existing_label.meta.time + edge.weight().travel_time).max(target.earliest_arrival);

            if (arrival_time > target.latest_arrival) {
                return  None
            }
            return Some (Label::extend(existing_label, edge, f64::from(edge.weight().cost), LabelMeta {
                time : arrival_time
            }))

        }
    }



    #[test]
    fn simple_check() {
        let mut solver = RcspSolver::new( SimpleProblem {});
        let paths = solver.find_paths::<false>(&[], 0.0);

        assert_eq!(paths.len(),2);
        assert_eq!(paths[0].0, 10.0);
        assert_eq!(paths[1].0, 11.0);
        dbg!(paths);

    }
}