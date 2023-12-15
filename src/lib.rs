#![warn(warnings)]
#![warn(clippy::all, clippy::pedantic)]
#![allow(non_upper_case_globals)]
#![allow(clippy::needless_return)]
#![allow(clippy::items_after_statements)]
#![allow(unused_variables, unused_imports, dead_code)]

use std::cell::Cell;
use std::cmp;
use std::cmp::Ordering;
use binary_heap_plus::BinaryHeap;
use std::fmt::{Debug};
use std::marker::PhantomData;
use petgraph::stable_graph::EdgeIndex;
use petgraph::stable_graph::EdgeReference;
use petgraph::stable_graph::StableGraph;
use petgraph::prelude::{EdgeRef, NodeIndex};
use typed_arena::Arena;

/// Shorthand for petgraph stable graph
pub type GraphType<NodeWeight, EdgeWeight> = StableGraph::<NodeWeight, EdgeWeight, petgraph::Directed, u32>;

/// Generates the initial root label
pub trait InitialLabelGenerator<T> {
    /// Return the LabelMeta for the initial root label
    fn default(problem : &T) -> Self;
}

/// EdgeWeight Trait
pub trait UserEdgeWeight : Debug {}

/// Main trait to implement that holds the user implementation of the problem.
///
/// Generics to implement:
/// - LabelMeta : Data to be stored in labels
/// - NodeWeight: Node attributes
/// - EdgeWeight: Edge attributes
/// - BranchFilter: Constraints made available during label extension
///
pub trait UserProblem<LabelMeta: Meta,  NodeWeight,EdgeWeight : UserEdgeWeight, BranchFilter>
{
    /// build your problem graph and return it
    ///
    /// is called once during initialization of solver and then stored
    fn create_graph(&mut self) -> ProblemGraph<NodeWeight, EdgeWeight>;

    /// Dominance function called in labeling algorithm.
    /// return true if label_a domiates label_b
    fn is_dominating(label_a: &Label<LabelMeta>, label_b: &Label<LabelMeta>, active_filters : &[BranchFilter]) -> bool;

    /// Label extension function
    ///
    /// Receives previous label, edge, node weights, branching filters, and reference to the sink node
    /// Return None if extension infeasible, otherwise return new Label.
    fn extend_label<'arena>(
        &self,
        existing_label : &'arena Label<'arena, LabelMeta>,
        edge : &EdgeReference<EdgeWeight>,
        source : &NodeWeight,
        target : &NodeWeight,
        active_filters : &[BranchFilter],
        sink : NodeIndex
    ) -> Option<Label<'arena,LabelMeta>>;

}

/// Compact struct containing dag and references to its source and sink node.
pub struct ProblemGraph<NodeWeight,EdgeWeight : UserEdgeWeight> {
    pub dag : GraphType<NodeWeight,EdgeWeight>,
    pub start : NodeIndex,
    pub end : NodeIndex
}

/// The solver structure holding the user problem and graph
pub struct RcspSolver<Problem,LabelMeta : Meta, NodeWeight,EdgeWeight , BranchFilter>
    where
        Problem : UserProblem<LabelMeta, NodeWeight, EdgeWeight,  BranchFilter>,
        EdgeWeight: UserEdgeWeight
{
    user_problem :  Problem,
    graph : ProblemGraph<NodeWeight,EdgeWeight>,
    _meta : PhantomData<LabelMeta>,
    _ew : PhantomData<EdgeWeight>,
    _nw : PhantomData<NodeWeight>,
    _ft : PhantomData<BranchFilter>
}

impl<Problem : UserProblem<LabelMeta,NodeWeight, EdgeWeight, BranchFilter>,LabelMeta : Meta, NodeWeight,EdgeWeight, BranchFilter>
    RcspSolver<Problem,LabelMeta, NodeWeight, EdgeWeight, BranchFilter>
where EdgeWeight: UserEdgeWeight
{

    pub fn new(mut problem : Problem) -> Self {

        let graph = problem.create_graph();
        Self {
            user_problem : problem,
            graph,
            _meta: PhantomData,
            _ew: PhantomData,
            _nw: PhantomData,
            _ft : PhantomData
        }
    }

    /// Returns reference to stored problem graph
    pub  fn get_graph(&self) -> &ProblemGraph<NodeWeight,EdgeWeight> {
        &self.graph
    }

    /// Returns mutable reference to stored problem graph
    pub  fn get_graph_mut(&mut self) -> &mut ProblemGraph<NodeWeight,EdgeWeight> {
        &mut self.graph
    }

    /// Returns mutable reference to stored problem
    pub  fn get_problem_mut(&mut self) -> &mut Problem {
        &mut self.user_problem
    }

    /// Returns reference to stored problem
    pub  fn get_problem(&self) -> &Problem {
        &self.user_problem
    }

}

#[derive(Clone, PartialEq)]
/// Solution object describing path between source and sink node
pub struct FeasiblePath {
    pub path:  Vec<(NodeIndex,Option<EdgeIndex>)>
}

impl Debug for FeasiblePath{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("\nFeasiblePath\n")?;
        f.write_str("Visiting: ")?;
        for (node,_edge) in &self.path {
            f.write_fmt(format_args!("{}-", node.index()))?;
        }
        f.write_str("\n")?;
        Ok(())
    }
}

/// Trait to be implemented by user for LabelMeta
pub trait Meta : Clone + Debug  {}




impl< Problem : UserProblem<LabelMeta,  NodeWeight,EdgeWeight, BranchFilter> ,LabelMeta : Meta +  InitialLabelGenerator<Problem>,  NodeWeight,EdgeWeight, BranchFilter>
    RcspSolver< Problem,LabelMeta,  NodeWeight,EdgeWeight, BranchFilter>
    where EdgeWeight: UserEdgeWeight,
          NodeWeight: Debug + Clone ,
{


    /// Primary function of solver. Finds RCSP for network and returns non dominated results
    ///
    /// Return object is vector of (path cost, path) tuples
    pub fn find_paths<const EARLY_EXIT: bool>(&mut self, filters : &[BranchFilter], path_start_cost : f64) -> Vec<(f64,FeasiblePath)> {


        let raw_problem_graph = &self.graph;

        let start_node = raw_problem_graph.start;
        let end_node = raw_problem_graph.end;

        let dag = &raw_problem_graph.dag;

        // apply resource constrained shortest path algorithm using labels
        let mut labels_at : Vec<Vec<&Label<LabelMeta>>> = vec![
            Vec::with_capacity(128); dag.node_count()
        ];

        debug_assert_eq!(start_node.index(), 0);

        let arena = Arena::new();

        let initial_label = arena.alloc(Label {
            node : start_node,
            extended_along_edge : None,
            parent : None,
            meta : LabelMeta::default(&self.user_problem),
            costs: path_start_cost,
            was_dominated : Cell::new(false)
        });


        let mut unprocessed_labels: BinaryHeap<&Label<LabelMeta>, _> = BinaryHeap::with_capacity_by(4096, |a : &&Label<LabelMeta>, b : &&Label<LabelMeta>| {
            // keep labels with small cost at front
            b.costs.partial_cmp(&a.costs).unwrap_or(Ordering::Equal)
        });

        unprocessed_labels.push(initial_label);
        'main: loop {

            let process_opt = unprocessed_labels.pop();

            // generate all direct possible child labels that are feasible
            let potential_next_labels  = match process_opt {
                Some(process) => {

                    if process.was_dominated.get() {
                        continue
                    }

                    dag.edges_directed(
                        process.node, petgraph::Direction::Outgoing).into_iter().filter_map(|edge : EdgeReference<_>| {

                        return self.user_problem.extend_label(process, &edge, &dag[edge.source()], &dag[edge.target()], filters, end_node)
                    })




                },
                None => break,
            };

            // process every possible label
            for new_label in potential_next_labels {

                if let Some(added_label) = Self::add_with_dominance_check(&mut labels_at[new_label.node.index()], &arena,new_label, filters) {
                    if EARLY_EXIT {
                        if added_label.node == end_node && added_label.costs < -0.000_001 {
                            break 'main;
                        }
                    }
                    unprocessed_labels.push(added_label);
                }
            }
        }


        // is sorted by cost, so first = best
        let labels_at_end =  &labels_at[end_node.index()];
        let mut output : Vec<(f64,FeasiblePath)> = labels_at_end.iter().map(|label| {
             let mut path = Vec::new();
             Self::get_parent(&mut path, label);

             path.reverse();


             (label.costs, FeasiblePath {
                 path: path
             })
         }).collect();


        output.sort_unstable_by(|a,b| a.0.partial_cmp(&b.0).unwrap_or(cmp::Ordering::Equal) );

        output
    }




    #[inline(always)]
    fn get_parent(nodes : &mut Vec<(NodeIndex,Option<EdgeIndex>)>, label : &Label<LabelMeta>)   {
        nodes.push((label.node, label.extended_along_edge));
        if let Some(parent) = label.parent {
            Self::get_parent(nodes, parent);
        }
    }


    /// Internal function to insert new label to label storage
    ///
    /// Thanks to Gerhard Hiermann for the implementation
    /// https://github.com/ghiermann
    ///
    /// The label storage `labels` is a list sorted by cost
    /// and shoud remain sorted.
    /// This function has three goals:
    /// - Detect if new label is dominated by existing. In this case terminate
    /// - Insert the new label at the correct position
    /// - Prune all existing labels that the new label dominates
    ///
    fn add_with_dominance_check<'arena>(
        labels: & mut Vec<&'arena Label<'arena, LabelMeta>>,
        arena : &'arena Arena<Label<'arena, LabelMeta>>,
        candidate: Label<'arena, LabelMeta>,
        active_filters : &[BranchFilter]
                                        ) -> Option<&'arena Label<'arena, LabelMeta>>
    {



        let mut i = 0;
        // phase eins. Wir checken ob das label selbst dominiert wird!


        while i < labels.len() {
            if labels[i].costs > candidate.costs {
                break;
            } else if Problem::is_dominating(labels[i], &candidate, active_filters) {
                return None;
            } else if labels[i].costs == candidate.costs {
                break;
            } else {
                i += 1;
            }
        }

        // No cheaper ones found -> not dominated for now
        // i = index where labels are first more expensive than before
        // Now just check if I dominate one of the more expensive ones
        let added_label = arena.alloc(candidate);

        let mut candidate = &*added_label;
        // Phase two.
        // Am I dominating another label?

        let s_idx = i; // Pointer to the first
        let mut tmp = if i == labels.len() {
            // End of the list - just add it and return
            // I'm at the end, not dominating anything else, so I just have to add it
            labels.push(candidate);
            return Some(added_label);
        } else if Problem::is_dominating(candidate, labels[i], active_filters) {
            // If I dominate the first one right away, discard it
            labels[i] = candidate; // overwrite directly
            None // Not a candidate for exchange because it's immediately excluded

        } else {
            // If I don't dominate it: Insert label at the position
            std::mem::swap(&mut labels[i], &mut candidate);
            Some(candidate) // Save original s in tmp
        };


        let mut removed = 0;
        i += 1; // The first one was examined specifically, now moving on to the next one
        while i < labels.len() {


      // if reference dominates the current one
            if Problem::is_dominating(labels[s_idx], labels[i], active_filters) {


             // if I have saved another one
                if tmp.is_some() {
                 // Replace i with the one in memory
                    let val = tmp.take().unwrap();
                // Save my tmp instead of the dominated one

                    labels[i] = val;
                } else {
                    // Otherwise, it has to go
                    // (I have nothing to swap it with)

                    removed += 1;
                }
            } else {// If that does not dominate
                if tmp.is_some() {
                    // If I still have one from the tmp, exchange yours
                    // and put it in tmp

                    std::mem::swap(&mut labels[i], tmp.as_mut().unwrap());
                } else if removed > 0 {
                    // If I already have one as tmp to remove,
                    // move it
                    labels.swap(i, i - removed);
                }
            }
            i += 1;
        }
        // If there's still one left now, then add it at the end
        if let Some(label) = tmp {
            labels.push(label);
        } else if removed > 0 {
            let kept = labels.len() - removed;
            // Update label value
            for i in labels.iter().skip(kept) {
                i.was_dominated.set(true);
            }

            unsafe {
                labels.set_len(kept);
            }
        }
        
        return  Some(added_label)

    }


}





#[derive(Clone, Debug)]
/// Internal label struct used during algorithm
/// Holds cost used for domination and
///     LabelMeta used for resource tracking
pub struct Label<'a, M : Meta> {
    pub costs: f64,
    pub node : NodeIndex,
    pub extended_along_edge : Option<EdgeIndex>,
    pub parent : Option<&'a Label<'a,M>>,
    pub meta : M,
    was_dominated : Cell<bool>,
}


impl<'a,M : Meta> Label<'a, M> {

    /// Helper function that creates a child label from an existing label
    pub fn extend<T>( parent : &'a Label<'a,M>,edge : &EdgeReference<T>, delta_cost : f64, new_meta : M ) -> Self {
        Self {
            costs: parent.costs + delta_cost ,
            node: edge.target(),
            extended_along_edge: Some(edge.id()),
            parent : Some(parent),
            meta: new_meta,
            was_dominated: Cell::new(false)
        }
    }

    /// Label constructor
    pub fn new( cost : f64, node : NodeIndex,extended_along_edge : Option<EdgeIndex>, parent : Option<&'a Label<'a,M>>, meta : M) -> Self {
        Self{
            costs: cost,
            node,
            extended_along_edge,
            parent,
            meta,
            was_dominated : Cell::new(false)
        }
    }



}






