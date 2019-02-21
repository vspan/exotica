import Tkinter

__all__=['Tuning']

class Tuning(object):

    def __init__(self, problem):

        # Initial setup
        self.master = Tkinter.Tk()
        self.problem = problem

        # Grab current rhos
        self.rho = {k: problem.get_rho(k) for k in problem.get_task_maps().keys()}

        # Setup labels and entries 
        self.entries = {}
        for i, k in enumerate(problem.get_task_maps().keys()):
            Tkinter.Label(self.master, text=k).grid(row=i)
            self.entries[k] = Tkinter.Entry(self.master)
            self.entries[k].grid(row=i, column=1)
            self.entries[k].insert(0, self.problem.get_rho(k))

        # Setup buttons
        Tkinter.Button(self.master, text="Set", command=self.set_rho_into_task_maps).grid(row=len(self.entries), column=0, pady=4)
        Tkinter.Button(self.master, text="Quit", command=self.master.quit).grid(row=len(self.entries), column=1, pady=4)

    def mainloop(self):
        Tkinter.mainloop()

    def set_rho_into_task_maps(self):
        for k in self.problem.get_task_maps().keys():
            self.problem.set_rho(k, float(self.entries[k].get()))
