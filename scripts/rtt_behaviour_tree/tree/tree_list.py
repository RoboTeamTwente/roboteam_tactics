import json
import os

from tree import behaviour_tree


class TreeList:

    def __init__(self):
        self._trees = {}


    def load_trees(self, tree_dir):
        """
        Loads all behaviour trees from the json files found in the given tree_dir.
        """
        for file_name in os.listdir(tree_dir):
            if file_name.endswith(".json"):
                with open(tree_dir + file_name) as data_file:
                    data = json.load(data_file)

                    tree = behaviour_tree.BehaviourTree()
                    tree.load_from_json(data)
                    self._trees[tree.title] = tree


    def title_list(self):
        """
        Returns a list of all tree titles.
        """
        titles = []
        for key, tree in self._trees.iteritems():
            titles.append(tree.title)
        return titles


    def get_tree(self, title):
        """
        Returns the tree with the given title.
        If there is no such tree, returns None.
        """
        try:
            tree = self._trees[title]
        except KeyError:
            print "No such tree: " + title
            tree = None

        return tree
