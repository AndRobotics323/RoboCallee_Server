
from django.db import models



# Create your models here.


class Product(models.Model):

    modify_date = models.DateTimeField(null=True, blank=True)

    name = models.CharField(max_length=200)
    photo = models.ImageField(upload_to='product_images/', null=True, blank=True)
    description = models.TextField()
    create_date = models.DateTimeField()
    
    price = models.PositiveIntegerField()
    stock = models.PositiveIntegerField(default=0)  # 🔥 재고 수량

    def __str__(self):
        return f"{self.name} (재고: {self.stock})"


    # modify_date = models.DateTimeField(null=True, blank=True)

    # def __str__(self):
    #     return self.subject
