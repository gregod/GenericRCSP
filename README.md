# Generic RCSP
A rust library for solving the resource constrained shortest path problem using a dynamic programming labeling algorithm.
Was developed for solving pricing problems for branch-and-price.

## Usage:

Implement the trait ``UserProblem``. Define the label data, resource extension function, and dominance function.

```rust
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
```

## License 
Copyright (C) 2023 Gregor Godbersen
Copyright (C) 2023 Gerhard Hiermann

This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/.